// Copyright (C) 2020  Max Coutte, Gabriel Combe
// Copyright (C) 2020  Relativty.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma comment(lib, "Ws2_32.lib")
#pragma comment (lib, "Setupapi.lib")
#pragma comment(lib, "User32.lib")

#include <atomic>
#include <WinSock2.h>
#include <Windows.h>
#include "hidapi/hidapi.h"
#include "openvr_driver.h"
#include <codecvt>
#include <nlohmann/json.hpp>

#include "driverlog.h"

#include "Relativty_HMDDriver.hpp"
#include "Relativty_ServerDriver.hpp"
#include "Relativty_EmbeddedPython.h"
#include "Relativty_components.h"
#include "Relativty_base_device.h"

#include "Relativty_SocketServer.hpp"

#include <errno.h>
#include <string>

inline const char* const BoolToString(bool b)
{
	return b ? "true" : "false";
}

inline vr::HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z) {
	vr::HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

inline void Normalize(float norma[3], float v[3], float max[3], float min[3], int up, int down, float scale[3], float offset[3]) {
	for (int i = 0; i < 4; i++) {
		norma[i] = (((up - down) * ((v[i] - min[i]) / (max[i] - min[i])) + down) / scale[i])+ offset[i];
	}
}

std::wstring stringToWstring(const std::string& t_str)
{
	//setup converter
	typedef std::codecvt_utf8<wchar_t> convert_type;
	std::wstring_convert<convert_type, wchar_t> converter;

	//use converter (.to_bytes: wstr->str, .from_bytes: str->wstr)
	return converter.from_bytes(t_str);
}

vr::EVRInitError Relativty::HMDDriver::Activate(uint32_t unObjectId) {
	RelativtyDevice::Activate(unObjectId);
	this->setProperties();
	if (this->bIsStaticRotation) {
		Relativty::ServerDriver::Log("HMD set to static Rotation mode. no connections will be opened\n");
	}
	else if (this->bIsSerialComport) {
		openCom();
	} else {
		int result;
		result = hid_init(); //Result should be 0.
		if (result) {
			Relativty::ServerDriver::Log("USB: HID API initialization failed. \n");
			return vr::VRInitError_Driver_TrackedDeviceInterfaceUnknown;
		}

		this->handle = hid_open((unsigned short)m_iVid, (unsigned short)m_iPid, NULL);
		if (!this->handle) {
			#ifdef DRIVERLOG_H
			DriverLog("USB: Unable to open HMD device with pid=%d and vid=%d.\n", m_iPid, m_iVid);
			#else
			Relativty::ServerDriver::Log("USB: Unable to open HMD device with pid=" + std::to_string(m_iPid) + " and vid=" + std::to_string(m_iVid) + ".\n");
			#endif
			return vr::VRInitError_Init_InterfaceNotFound;
		}
	}

	this->retrieve_quaternion_isOn = true;
	this->retrieve_quaternion_thread_worker = std::thread(&Relativty::HMDDriver::retrieve_device_quaternion_packet_threaded, this);
	

	if (this->start_tracking_server) {
		this->retrieve_vector_isOn = true;
		this->retrieve_vector_thread_worker = std::thread(&Relativty::HMDDriver::retrieve_client_vector_packet_threaded, this);
		while (this->serverNotReady) {
			// do nothing
		}
		//this->startPythonTrackingClient_worker = std::thread(startPythonTrackingClient_threaded, this->PyPath);
	}

	this->update_pose_thread_worker = std::thread(&Relativty::HMDDriver::update_pose_threaded, this);

	return vr::VRInitError_None;
}

void Relativty::HMDDriver::closeCom() {
	if (bIsStaticRotation)
		return;
	CloseHandle(this->serialHandle);
	this->serialHandle = NULL;
}

void Relativty::HMDDriver::openCom() {
	if (bIsStaticRotation)
		return;
	serialHandle = CreateFileW(comport.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (serialHandle == INVALID_HANDLE_VALUE) {
		Relativty::ServerDriver::Log("COM: COM initialization failed. \"" + comportRAW + "\"\n");
	}
	else {
		Relativty::ServerDriver::Log("COM: COM opened. \n");
		// Do some basic settings
		DCB serialParams = { 0 };
		serialParams.DCBlength = sizeof(serialParams);

		GetCommState(serialHandle, &serialParams);
		serialParams.BaudRate = 9600;
		serialParams.ByteSize = 8;
		serialParams.StopBits = ONESTOPBIT;
		serialParams.Parity = PARITY_NONE;
		SetCommState(serialHandle, &serialParams);

		// Set timeouts
		COMMTIMEOUTS timeout = { 0 };
		timeout.ReadIntervalTimeout = 50;
		timeout.ReadTotalTimeoutConstant = 50;
		timeout.ReadTotalTimeoutMultiplier = 50;
		timeout.WriteTotalTimeoutConstant = 50;
		timeout.WriteTotalTimeoutMultiplier = 10;

		SetCommTimeouts(serialHandle, &timeout);
	}
}

//Connect to the designated blob Tracker Pi-Device
//One device only sofar.
void Relativty::HMDDriver::openSocket()
{

}

void Relativty::HMDDriver::closeSocket()
{
}

void Relativty::HMDDriver::Deactivate() {
	this->retrieve_quaternion_isOn = false;
	this->retrieve_quaternion_thread_worker.join();
	if (this->bIsSerialComport) {
		closeCom();
	} else {
		hid_close(this->handle);
		hid_exit();
	}
	

	if (this->start_tracking_server) {
		this->retrieve_vector_isOn = false;
		closesocket(this->sock);
		this->retrieve_vector_thread_worker.join();
		WSACleanup();
	}
	RelativtyDevice::Deactivate();
	this->update_pose_thread_worker.join();

	Relativty::ServerDriver::Log("Thread0: all threads exit correctly \n");
}

void Relativty::HMDDriver::update_pose_threaded() {
	Relativty::ServerDriver::Log("Thread2: successfully started\n");
	while (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
		if (this->new_quaternion_avaiable && this->new_vector_avaiable) {
			m_Pose.qRotation.w = this->quat[0];
			m_Pose.qRotation.x = this->quat[1];
			m_Pose.qRotation.y = this->quat[2];
			m_Pose.qRotation.z = this->quat[3];

			m_Pose.vecPosition[0] = this->vector_xyz[0];
			m_Pose.vecPosition[1] = this->vector_xyz[1];
			m_Pose.vecPosition[2] = this->vector_xyz[2];

			m_Pose.vecDriverFromHeadTranslation[0] = double(10);
			m_Pose.vecDriverFromHeadTranslation[1] = double(0);
			m_Pose.vecDriverFromHeadTranslation[2] = double(0);

			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_quaternion_avaiable = false;
			this->new_vector_avaiable = false;

		} else if (this->new_quaternion_avaiable) {
			m_Pose.qRotation.w = this->quat[0];
			m_Pose.qRotation.x = this->quat[1];
			m_Pose.qRotation.y = this->quat[2];
			m_Pose.qRotation.z = this->quat[3];
			//Relativty::ServerDriver::Log("ARCH|\tQuartonian Received\n");
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_quaternion_avaiable = false;

		} else if (this->new_vector_avaiable) {

			m_Pose.vecPosition[0] = this->vector_xyz[0];
			m_Pose.vecPosition[1] = this->vector_xyz[1];
			m_Pose.vecPosition[2] = this->vector_xyz[2];
			Relativty::ServerDriver::Log("ARCH|\tVector Received\n");
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_vector_avaiable = false;

		}
	}
	Relativty::ServerDriver::Log("Thread2: successfully stopped\n");
}

void Relativty::HMDDriver::calibrate_quaternion() {
	if ((0x01 & GetAsyncKeyState(0x52)) != 0) {
		qconj[0].store(quat[0]);
		qconj[1].store(-1 * quat[1]);
		qconj[2].store(-1 * quat[2]);
		qconj[3].store(-1 * quat[3]);
	}
	float qres[4];

	qres[0] = qconj[0] * quat[0] - qconj[1] * quat[1] - qconj[2] * quat[2] - qconj[3] * quat[3];
	qres[1] = qconj[0] * quat[1] + qconj[1] * quat[0] + qconj[2] * quat[3] - qconj[3] * quat[2];
	qres[2] = qconj[0] * quat[2] - qconj[1] * quat[3] + qconj[2] * quat[0] + qconj[3] * quat[1];
	qres[3] = qconj[0] * quat[3] + qconj[1] * quat[2] - qconj[2] * quat[1] + qconj[3] * quat[0];

	this->quat[0] = qres[0];
	this->quat[1] = qres[1];
	this->quat[2] = qres[2];
	this->quat[3] = qres[3];
}

void Relativty::HMDDriver::retrieve_device_quaternion_packet_threaded() {
	uint8_t packet_buffer[64];
	int16_t quaternion_packet[4];
	//this struct is for mpu9250 support
	#pragma pack(push, 1)
	struct pak {
		uint8_t id;
		float quat[4];
		uint8_t rest[47];
	};
	#pragma pack(pop)
	int result;
	Relativty::ServerDriver::Log("Thread1: successfully started\n");
	while (this->retrieve_quaternion_isOn) {
		if (this->bIsSerialComport) {
			//confirm we are connected. if not. instead ake an attempt to connect and forfiet this read cycle.

			//read some serial in :
			
			std::string jsonString = "";
			if (!this->bIsStaticRotation) {
				//Relativty::ServerDriver::Log("ARCH| Begin read in of JSON packet :\n");
				bool reading = true;
				bool awaitStart = true;
				bool awaitEnd = false;
				int attempt = 64;

				//read until "{" - discard any others
				//then read until "}"
				while (reading) {
					attempt--;
					if (attempt <= 0) {
						jsonString = ""; // we failed. clear the buffer
						break;
					}
					unsigned char readBuffer;
					DWORD byteRead;
					//Relativty::ServerDriver::Log("ARCH| read next\n");
					result = ReadFile(serialHandle, &readBuffer, 1, &byteRead, NULL);
					int err = GetLastError();
					if (err != ERROR_SUCCESS) {
						Relativty::ServerDriver::Log("ARCH| HMD| COM| readFile error [" + std::to_string(err) + "]");
						closeCom();
						openCom(); //attempt to reconnect
						break;
					}
					if (result) {
						std::string asString(1, readBuffer);
						//Relativty::ServerDriver::Log("ARCH| read one '" + asString + "'\n");
						if (awaitStart && asString == "{") {
							jsonString += asString;
							awaitStart = false;
							awaitEnd = true;
						}
						else if (awaitEnd) {
							jsonString += asString;
							if (asString == "}") {
								reading = false;
								break;
							}
						}
					}
				}
			}
			else {
				jsonString = "{\"W\":1.00,\"X\":0.00,\"Y\":0.00,\"Z\":0.00}";
			}
			//Relativty::ServerDriver::Log("ARCH| read packet built...\n");
			//readBuffer[byteReading] = 0; //0 off the bytes
			if (jsonString.size() > 0) {
				//Relativty::ServerDriver::Log("ARCH| read in com as string : \"" + jsonString + "\"\n");
				try {
					nlohmann::json json = nlohmann::json::parse(jsonString);
					//Relativty::ServerDriver::Log("ARCH| read in json pretty value as '" + json.dump() + "'\n");
					//Relativty::ServerDriver::Log("ARCH| read in json X as '" + std::to_string(json["X"].get<float>()) + "'\n");
					//Relativty::ServerDriver::Log("ARCH| read in json Y as '" + std::to_string(json["Y"].get<float>()) + "'\n");
					//Relativty::ServerDriver::Log("ARCH| read in json Z as '" + std::to_string(json["Z"].get<float>()) + "'\n");
					//Relativty::ServerDriver::Log("ARCH| read in json W as '" + std::to_string(json["W"].get<float>()) + "'\n");
					//Relativty::ServerDriver::Log("ARCH| assign in the quat '" + json.dump() + "'\n");

					this->quat[0] = json["W"].get<float>();
					this->quat[1] = json["X"].get<float>() * -1; // pitch
					this->quat[2] = json["Z"].get<float>();
					this->quat[3] = json["Y"].get<float>();
					//Relativty::ServerDriver::Log("ARCH| have assigned in the quat '" + json.dump() + "'\n");

					this->calibrate_quaternion();

					this->new_quaternion_avaiable = true;
				}
				catch (nlohmann::json::parse_error& e)
				{
					Relativty::ServerDriver::Log("ARCH| read in json failed parse '" + std::string(e.what()) + "'\n");
				}
				
				
			} else {
				const char* strr = BoolToString(bIsStaticRotation);

				Relativty::ServerDriver::Log("ARCH| read in com as string Failed. bIsStaticRotation [");
				Relativty::ServerDriver::Log(strr);
				Relativty::ServerDriver::Log("]\n");
				//we can try to open the port again ?

			}
		} else {
			result = hid_read(this->handle, packet_buffer, 64); //Result should be greater than 0.
			if (result > 0) {


				if (m_bIMUpktIsDMP) {

					quaternion_packet[0] = ((packet_buffer[1] << 8) | packet_buffer[2]);
					quaternion_packet[1] = ((packet_buffer[5] << 8) | packet_buffer[6]);
					quaternion_packet[2] = ((packet_buffer[9] << 8) | packet_buffer[10]);
					quaternion_packet[3] = ((packet_buffer[13] << 8) | packet_buffer[14]);
					this->quat[0] = static_cast<float>(quaternion_packet[0]) / 16384.0f;
					this->quat[1] = static_cast<float>(quaternion_packet[1]) / 16384.0f;
					this->quat[2] = static_cast<float>(quaternion_packet[2]) / 16384.0f;
					this->quat[3] = static_cast<float>(quaternion_packet[3]) / 16384.0f;

					float qres[4];
					qres[0] = quat[0];
					qres[1] = quat[1];
					qres[2] = -1 * quat[2];
					qres[3] = -1 * quat[3];

					this->quat[0] = qres[0];
					this->quat[1] = qres[1];
					this->quat[2] = qres[2];
					this->quat[3] = qres[3];

					this->calibrate_quaternion();

					this->new_quaternion_avaiable = true;

				}
				else {

					pak* recv = (pak*)packet_buffer;
					this->quat[0] = recv->quat[0];
					this->quat[1] = recv->quat[1];
					this->quat[2] = recv->quat[2];
					this->quat[3] = recv->quat[3];

					this->calibrate_quaternion();

					this->new_quaternion_avaiable = true;

				}


			}
			else {
				Relativty::ServerDriver::Log("Thread1: Issue while trying to read USB\n");
			}
		} 
	}
	Relativty::ServerDriver::Log("Thread1: successfully stopped\n");
}

void Relativty::HMDDriver::retrieve_client_vector_packet_threaded() {
	WSADATA wsaData;
	struct sockaddr_in server, client;
	int addressLen;
	int receiveBufferLen = 128;
	char receiveBuffer[128];
	int resultReceiveLen;

	float normalize_min[3]{ this->normalizeMinX, this->normalizeMinY, this->normalizeMinZ};
	float normalize_max[3]{ this->normalizeMaxX, this->normalizeMaxY, this->normalizeMaxZ};
	float scales_coordinate_meter[3]{ this->scalesCoordinateMeterX, this->scalesCoordinateMeterY, this->scalesCoordinateMeterZ};
	float offset_coordinate[3] = { this->offsetCoordinateX, this->offsetCoordinateY, this->offsetCoordinateZ};

	//float coordinate[3]{ 0,0,0 };


	float coordinate_normalized[3];

	Relativty::ServerDriver::Log("Thread3: Initialising Socket Server.\n");


	//if (!bIsStaticPosition) {
	//}

	this->serverNotReady = false;

	Relativty::ServerDriver::Log("Thread3: successfully started\n");
	while (this->retrieve_vector_isOn) {
		if (bIsStaticPosition) {
			continue;
		}
		//get blue coordinates :
		Relativty::SocketServer sss;
		SocketServer::deviceState state = sss.getState('B');
		float *coordinate = state.coordinate;
		//if (applyCoordinates) {
		//float *coordinate = state.coordinate;
		const float mod = fScaleBy;
		Relativty::ServerDriver::Log("ARCH| SOCKET| scale by " + std::to_string(fScaleBy));
		coordinate[0] = coordinate[0] * mod;
		coordinate[1] = coordinate[1] * mod;
		coordinate[2] = coordinate[2] * mod;
		Relativty::ServerDriver::Log("ARCH| SOCKET| normie");
		Normalize(coordinate_normalized, coordinate, normalize_max, normalize_min, this->upperBound, this->lowerBound, scales_coordinate_meter, offset_coordinate);
		this->vector_xyz[0] = coordinate_normalized[1];
		this->vector_xyz[1] = coordinate_normalized[2];
		this->vector_xyz[2] = coordinate_normalized[0];
		this->new_vector_avaiable = true;
		//}
			
		//quaternion here :
		//this->quat[0] = json["qw"].get<float>();
		//this->quat[1] = json["qx"].get<float>() * -1; // pitch
		//this->quat[2] = json["qz"].get<float>();
		//this->quat[3] = json["qy"].get<float>();
		//Relativty::ServerDriver::Log("ARCH| have assigned in the quat '" + json.dump() + "'\n");

		//this->calibrate_quaternion();

		//this->new_quaternion_avaiable = true;
	}
	Relativty::ServerDriver::Log("Thread3: successfully stopped\n");
}

Relativty::HMDDriver::HMDDriver(std::string myserial):RelativtyDevice(myserial, "akira_") {
	// keys for use with the settings API
	static const char* const Relativty_hmd_section = "Relativty_hmd";
	// openvr api stuff
	m_sRenderModelPath = "{Relativty}/rendermodels/generic_hmd";
	m_sBindPath = "{Relativty}/input/relativty_hmd_profile.json";

	m_spExtDisplayComp = std::make_shared<Relativty::RelativtyExtendedDisplayComponent>();

	// not openvr api stuff
	Relativty::ServerDriver::Log("Loading Settings\n");
	this->IPD = vr::VRSettings()->GetFloat(Relativty_hmd_section, "IPDmeters");
	this->SecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(Relativty_hmd_section, "secondsFromVsyncToPhotons");
	this->DisplayFrequency = vr::VRSettings()->GetFloat(Relativty_hmd_section, "displayFrequency");

	this->start_tracking_server = vr::VRSettings()->GetBool(Relativty_hmd_section, "startTrackingServer");
	this->upperBound = vr::VRSettings()->GetFloat(Relativty_hmd_section, "upperBound");
	this->lowerBound = vr::VRSettings()->GetFloat(Relativty_hmd_section, "lowerBound");
	this->normalizeMinX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinX");
	this->normalizeMinY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinY");
	this->normalizeMinZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinZ");
	this->normalizeMaxX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxX");
	this->normalizeMaxY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxY");
	this->normalizeMaxZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxZ");
	this->scalesCoordinateMeterX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterX");
	this->scalesCoordinateMeterY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterY");
	this->scalesCoordinateMeterZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterZ");
	this->offsetCoordinateX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateX");
	this->offsetCoordinateY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateY");
	this->offsetCoordinateZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateZ");

	this->m_iPid = vr::VRSettings()->GetInt32(Relativty_hmd_section, "hmdPid");
	this->m_iVid = vr::VRSettings()->GetInt32(Relativty_hmd_section, "hmdVid");

	char comportBuffer[1024];
	vr::VRSettings()->GetString(Relativty_hmd_section, "comport", comportBuffer, sizeof(comportBuffer));
	this->comportRAW = comportBuffer;
	this->comport = stringToWstring(comportBuffer);

	this->m_bIMUpktIsDMP = vr::VRSettings()->GetBool(Relativty_hmd_section, "hmdIMUdmpPackets");

	this->bIsSerialComport = vr::VRSettings()->GetBool(Relativty_hmd_section, "isSerialComport");

	this->bIsStaticPosition = !vr::VRSettings()->GetBool(Relativty_hmd_section, "willTrackPosition");
	this->bIsStaticRotation = !vr::VRSettings()->GetBool(Relativty_hmd_section, "willTrackRotation");

	this->bIsTrackingA = vr::VRSettings()->GetBool(Relativty_hmd_section, "willTrackA");
	this->bIsTrackingB = vr::VRSettings()->GetBool(Relativty_hmd_section, "willTrackB");
	this->fScaleBy = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scaleBy");

	char buffer[1024];
	vr::VRSettings()->GetString(Relativty_hmd_section, "PyPath", buffer, sizeof(buffer));
	this->PyPath = buffer;

	// this is a bad idea, this should be set by the tracking loop
	m_Pose.result = vr::TrackingResult_Running_OK;
}

inline void Relativty::HMDDriver::setProperties() {
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserIpdMeters_Float, this->IPD);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.16f);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DisplayFrequency_Float, this->DisplayFrequency);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_SecondsFromVsyncToPhotons_Float, this->SecondsFromVsyncToPhotons);

	// avoid "not fullscreen" warnings from vrmonitor
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_IsOnDesktop_Bool, false);
}

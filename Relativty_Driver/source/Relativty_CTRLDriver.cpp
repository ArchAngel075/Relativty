// Copyright (C) 2020  Jaco Kotze, modified from HMD
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

#include "Relativty_CTRLDriver.hpp"
#include "Relativty_ServerDriver.hpp"
#include "Relativty_EmbeddedPython.h"
#include "Relativty_components.h"
#include "Relativty_base_device.h"


#include <string>

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

std::wstring stringToWstringCTRL(const std::string& t_str)
{
	//setup converter
	typedef std::codecvt_utf8<wchar_t> convert_type;
	std::wstring_convert<convert_type, wchar_t> converter;

	//use converter (.to_bytes: wstr->str, .from_bytes: str->wstr)
	return converter.from_bytes(t_str);
}

void Relativty::CTRLDriver::closeCom() {
	if (bIsStaticRotation)
		return;
	CloseHandle(this->serialHandle);
	this->serialHandle = NULL;
}

void Relativty::CTRLDriver::openCom() {
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

vr::EVRInitError Relativty::CTRLDriver::Activate(uint32_t unObjectId) {
	Relativty::ServerDriver::Log("CTRL| Booting...\n");
	RelativtyDevice::Activate(unObjectId);
	Relativty::ServerDriver::Log("CTRL| parent activated\n");
	this->setProperties();
	Relativty::ServerDriver::Log("CTRL| properties set\n");
	if (this->bIsStaticRotation) {
		Relativty::ServerDriver::Log("CTRL| set to static Rotation mode. no connections will be opened\n");
	}
	else if (this->bIsSerialComport) {
		openCom();
	}
	else {
		int result;
		result = hid_init(); //Result should be 0.
		if (result) {
			Relativty::ServerDriver::Log("USB: HID API initialization failed. \n");
			return vr::VRInitError_Driver_TrackedDeviceInterfaceUnknown;
		}

		this->handle = hid_open((unsigned short)m_iVid, (unsigned short)m_iPid, NULL);
		if (!this->handle) {
#ifdef DRIVERLOG_H
			DriverLog("USB: Unable to open CTRL device with pid=%d and vid=%d.\n", m_iPid, m_iVid);
#else
			Relativty::ServerDriver::Log("USB: Unable to open CTRL device with pid=" + std::to_string(m_iPid) + " and vid=" + std::to_string(m_iVid) + ".\n");
#endif
			return vr::VRInitError_Init_InterfaceNotFound;
		}
	}

	this->retrieve_quaternion_isOn = true;
	this->retrieve_quaternion_thread_worker = std::thread(&Relativty::CTRLDriver::retrieve_device_quaternion_packet_threaded, this);

	if (this->start_tracking_server) {
		this->retrieve_vector_isOn = true;
		this->retrieve_vector_thread_worker = std::thread(&Relativty::CTRLDriver::retrieve_client_vector_packet_threaded, this);
		while (this->serverNotReady) {
			// do nothing
		}
		//this->startPythonTrackingClient_worker = std::thread(startPythonTrackingClient_threaded, this->PyPath);
	}

	this->update_pose_thread_worker = std::thread(&Relativty::CTRLDriver::update_pose_threaded, this);
		
	return vr::VRInitError_None;
}

void Relativty::CTRLDriver::Deactivate() {
	this->retrieve_quaternion_isOn = false;
	this->retrieve_quaternion_thread_worker.join();
	if (this->bIsSerialComport) {
		CloseHandle(this->serialHandle);
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

	Relativty::ServerDriver::Log("CTRL| CTRL| Thread0: all threads exit correctly \n");
}

void Relativty::CTRLDriver::update_pose_threaded() {
	Relativty::ServerDriver::Log("CTRL| CTRL| Thread2: successfully started\n");
	while (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
		if (this->new_quaternion_avaiable && this->new_vector_avaiable) {
			m_Pose.qRotation.w = this->quat[0];
			m_Pose.qRotation.x = this->quat[1];
			m_Pose.qRotation.y = this->quat[2];
			m_Pose.qRotation.z = this->quat[3];

			m_Pose.vecPosition[0] = this->vector_xyz[0];
			m_Pose.vecPosition[1] = this->vector_xyz[1];
			m_Pose.vecPosition[2] = this->vector_xyz[2];

			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_quaternion_avaiable = false;
			this->new_vector_avaiable = false;

		}
		else if (this->new_quaternion_avaiable) {
			m_Pose.qRotation.w = this->quat[0];
			m_Pose.qRotation.x = this->quat[1];
			m_Pose.qRotation.y = this->quat[2];
			m_Pose.qRotation.z = this->quat[3];
			//Relativty::ServerDriver::Log("ARCH|\tQuartonian Received\n");
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_quaternion_avaiable = false;

		}
		else if (this->new_vector_avaiable) {

			m_Pose.vecPosition[0] = this->vector_xyz[0];
			m_Pose.vecPosition[1] = this->vector_xyz[1];
			m_Pose.vecPosition[2] = this->vector_xyz[2];
			//Relativty::ServerDriver::Log("ARCH|\tVector Received\n");
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_vector_avaiable = false;

		}
	}
	Relativty::ServerDriver::Log("CTRL| Thread2: successfully stopped\n");
}

void Relativty::CTRLDriver::calibrate_quaternion() {
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

void Relativty::CTRLDriver::retrieve_device_quaternion_packet_threaded() {
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
						Relativty::ServerDriver::Log("ARCH| CTRL| COM| readFile error [" + std::to_string(err) + "]");
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
				jsonString = "{\"W\":1.00,\"X\":-0.03,\"Y\":0.01,\"Z\":0.03}";
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
					Relativty::ServerDriver::Log("ARCH| CTRL| read in json failed parse '" + std::string(e.what()) + "'\n");
				}


			}
			else {
				Relativty::ServerDriver::Log("ARCH| CTRL| read in com as string Failed");
				//we can try to open the port again ?

			}
		}
	}
	Relativty::ServerDriver::Log("CTRL| Thread1: successfully stopped\n");
}

void Relativty::CTRLDriver::retrieve_client_vector_packet_threaded() {
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

	float coordinate[3]{ 0, 0, 0 };
	float coordinate_normalized[3];

	Relativty::ServerDriver::Log("CTRL| Thread3: Initialising Socket.\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		Relativty::ServerDriver::Log("CTRL| Thread3: Failed. Error Code: " + WSAGetLastError());
		return;
	}
	Relativty::ServerDriver::Log("CTRL| Thread3: Socket successfully initialised.\n");

	if (!bIsStaticPosition && (this->sock = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
		Relativty::ServerDriver::Log("CTRL| Thread3: could not create socket: " + WSAGetLastError());
	Relativty::ServerDriver::Log("CTRL| Thread3: Socket created.\n");

	server.sin_family = AF_INET;
	server.sin_port = htons(50000);
	server.sin_addr.s_addr = INADDR_ANY;

	if (!bIsStaticPosition && bind(this->sock, (struct sockaddr*) & server, sizeof(server)) == SOCKET_ERROR)
		Relativty::ServerDriver::Log("CTRL| Thread3: Bind failed with error code: " + WSAGetLastError());
	Relativty::ServerDriver::Log("CTRL| Thread3: Bind done \n");

	if (!bIsStaticPosition)
		listen(this->sock, 1);

	this->serverNotReady = false;

	if (!bIsStaticPosition) {
		Relativty::ServerDriver::Log("Thread3: Waiting for incoming connections...\n");
		addressLen = sizeof(struct sockaddr_in);
		this->sock_receive = accept(this->sock, (struct sockaddr*) & client, &addressLen);
		if (this->sock_receive == INVALID_SOCKET)
			Relativty::ServerDriver::Log("Thread3: accept failed with error code: " + WSAGetLastError());
		Relativty::ServerDriver::Log("Thread3: Connection accepted");
	}

	Relativty::ServerDriver::Log("CTRL| Thread3: successfully started\n");
	while (this->retrieve_vector_isOn) {
		if (bIsStaticPosition) {
			continue;
		}
		resultReceiveLen = recv(this->sock_receive, receiveBuffer, receiveBufferLen, NULL);
		if (resultReceiveLen == -1) {
			fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
			Relativty::ServerDriver::Log("ARCH| CTRL| SOCK| recv error [" + std::to_string(errno) + "].");
			continue;
		}
		if (resultReceiveLen > 0) {
			std::string receiveString = std::string(receiveBuffer);
			Relativty::ServerDriver::Log("ARCH| CTRL| SOCKET| RAW Received '" + receiveString + "'\n");
			size_t opens = receiveString.find("{");
			if (opens != 0) {
				Relativty::ServerDriver::Log("ARCH| CTRL| SOCKET| Bad packet. Discarding'\n");
				continue;
			}
			size_t close = receiveString.find("}") + 1;
			receiveString.resize(close);
			//Relativty::ServerDriver::Log("ARCH| SOCKET| Received '" + receiveString + "'\n");
			nlohmann::json json = nlohmann::json::parse(receiveString);
			//Relativty::ServerDriver::Log("CTRL| ARCH| SOCKET| read in json pretty value as '" + json.dump() + "'\n");
			//Relativty::ServerDriver::Log("CTRL| ARCH| SOCKET| get x\n");
			coordinate[0] = json["px"].get<float>();
			//Relativty::ServerDriver::Log("CTRL| ARCH| SOCKET| get y\n");
			coordinate[1] = json["py"].get<float>();
			//Relativty::ServerDriver::Log("CTRL| ARCH| SOCKET| get z\n");
			coordinate[2] = json["pz"].get<float>();
			Normalize(coordinate_normalized, coordinate, normalize_max, normalize_min, this->upperBound, this->lowerBound, scales_coordinate_meter, offset_coordinate);
			this->vector_xyz[0] = -coordinate_normalized[1];
			this->vector_xyz[1] = -coordinate_normalized[2];
			this->vector_xyz[2] = -coordinate_normalized[0];
			this->new_vector_avaiable = true;

			//key binds here :
			//Relativty::ServerDriver::Log("CTRL| ARCH| SOCKET| get buttons\n");
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[0], json["b0"].get<int>() == 1, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[1], json["b1"].get<int>() == 1, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[2], json["b2"].get<int>() == 1, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], json["b3"].get<int>() == 1, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], json["b4"].get<int>() == 1, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], json["b5"].get<int>() == 1, 0);

			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[0], json["b6"].get<float>(), 0); //Trackpad x
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[1], json["b7"].get<float>(), 0); //Trackpad y
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[2], json["b8"].get<float>(), 0); //Trigger

			//quaternion here :
			//this->quat[0] = json["qw"].get<float>();
			//this->quat[1] = json["qx"].get<float>() * -1; // pitch
			//this->quat[2] = json["qz"].get<float>();
			//this->quat[3] = json["qy"].get<float>();
			//Relativty::ServerDriver::Log("CTRL| ARCH| have assigned in the quat '" + json.dump() + "'\n");

			//this->calibrate_quaternion();

			//this->new_quaternion_avaiable = true;
		}
	}
	Relativty::ServerDriver::Log("CTRL| Thread3: successfully stopped\n");

}

Relativty::CTRLDriver::CTRLDriver(std::string myserial):RelativtyDevice(myserial, "akira_") {
	// keys for use with the settings API
	static const char* const Relativty_ctrl_section = "Relativty_ctrl";

	// openvr api stuff
	m_sRenderModelPath = "C:\\Users\\Arch\\Documents\\SteamVR\\Relativty\Relativty\\Relativty_Driver\\Relativty\\resources\\rendermodels\\generic_hmd";
	m_sBindPath = "{Relativty}/input/relativty_hmd_profile.json";

	m_spExtDisplayComp = std::make_shared<Relativty::RelativtyExtendedDisplayComponent>();

	// not openvr api stuff
	Relativty::ServerDriver::Log("CTRL| Loading Settings\n");
	this->IPD = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "IPDmeters");
	this->SecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "secondsFromVsyncToPhotons");
	this->DisplayFrequency = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "displayFrequency");

	this->start_tracking_server = vr::VRSettings()->GetBool(Relativty_ctrl_section, "startTrackingServer");
	this->upperBound = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "upperBound");
	this->lowerBound = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "lowerBound");
	this->normalizeMinX = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "normalizeMinX");
	this->normalizeMinY = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "normalizeMinY");
	this->normalizeMinZ = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "normalizeMinZ");
	this->normalizeMaxX = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "normalizeMaxX");
	this->normalizeMaxY = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "normalizeMaxY");
	this->normalizeMaxZ = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "normalizeMaxZ");
	this->scalesCoordinateMeterX = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "scalesCoordinateMeterX");
	this->scalesCoordinateMeterY = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "scalesCoordinateMeterY");
	this->scalesCoordinateMeterZ = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "scalesCoordinateMeterZ");
	this->offsetCoordinateX = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "offsetCoordinateX");
	this->offsetCoordinateY = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "offsetCoordinateY");
	this->offsetCoordinateZ = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "offsetCoordinateZ");

	this->m_iPid = vr::VRSettings()->GetInt32(Relativty_ctrl_section, "ctrlPid");
	this->m_iVid = vr::VRSettings()->GetInt32(Relativty_ctrl_section, "ctrlVid");

	char comportBuffer[1024];
	vr::VRSettings()->GetString(Relativty_ctrl_section, "comport", comportBuffer, sizeof(comportBuffer));
	this->comportRAW = comportBuffer;
	this->comport = stringToWstringCTRL(comportBuffer);

	this->m_bIMUpktIsDMP = vr::VRSettings()->GetBool(Relativty_ctrl_section, "ctrlIMUdmpPackets");

	this->bIsSerialComport = vr::VRSettings()->GetBool(Relativty_ctrl_section, "isSerialComport");

	this->bIsStaticPosition = !vr::VRSettings()->GetBool(Relativty_ctrl_section, "willTrackPosition");
	this->bIsStaticRotation = !vr::VRSettings()->GetBool(Relativty_ctrl_section, "willTrackRotation");

	char buffer[1024];
	vr::VRSettings()->GetString(Relativty_ctrl_section, "PyPath", buffer, sizeof(buffer));
	this->PyPath = buffer;

	// this is a bad idea, this should be set by the tracking loop
	m_Pose.result = vr::TrackingResult_Running_OK;
}

inline void Relativty::CTRLDriver::setProperties() {
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserIpdMeters_Float, this->IPD);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.16f);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DisplayFrequency_Float, this->DisplayFrequency);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_SecondsFromVsyncToPhotons_Float, this->SecondsFromVsyncToPhotons);

	// avoid "not fullscreen" warnings from vrmonitor
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_IsOnDesktop_Bool, false);

	//bindings :
	/*
		0 f
		1 g
		2 h
		3 j
		4 k
		5 l
	*/
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &HButtons[0]); //f
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtons[2]); //h

	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &HButtons[1]); //g
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtons[5]); //l

	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &HButtons[3]); //j
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtons[4]); //k

	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &HAnalog[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &HAnalog[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalog[2], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
}

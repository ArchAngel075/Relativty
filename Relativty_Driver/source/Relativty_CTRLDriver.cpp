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

#include <Eigen/Geometry>

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

inline Eigen::Vector3f getControllerTrueCenter(Eigen::Vector3f controllerTrackedPosition, float* controllerRotationIn, Eigen::Vector3f forwardAxis, float distance) {
	using Eigen::Quaternion;
	using Eigen::Vector3;
	using Eigen::Matrix3;
	Eigen::Quaternion<float> q_IMU { controllerRotationIn[0],controllerRotationIn[1], controllerRotationIn[2],controllerRotationIn[3] };
	Eigen::Vector3f contrllerTrackedPositionVector{ controllerTrackedPosition[1],controllerTrackedPosition[2],controllerTrackedPosition[0] };
	Eigen::Vector3f forwardAxisVector { forwardAxis[0],forwardAxis[1],forwardAxis[2] };
	//forwardAxisVector.normalize();
	Relativty::ServerDriver::Log("CTRL IMU axis angle and distance to blob is (" + std::to_string(forwardAxis[0]) + ", " + std::to_string(forwardAxis[1]) + ", " + std::to_string(forwardAxis[2]) + ") @ " + std::to_string(distance) +"\n");

	Eigen::Vector3f LED_forward_vector = q_IMU * forwardAxisVector;
	Eigen::Vector3f Assumed_position = contrllerTrackedPositionVector + (distance * LED_forward_vector);
	//positionInVector3.y() is LR
	//positionInVector3.z(); is up
	// positionInVector3.x() is fb
	//float newX = Assumed_position.y();// //lr
	//float newY = Assumed_position.z();// //ud
	//float newZ = Assumed_position.x();// //fb

	float newX = Assumed_position.x();// //lr
	float newY = Assumed_position.y();// //ud
	float newZ = Assumed_position.z();// //fb
	return Eigen::Vector3f(newX, newY, newZ);
	//return Eigen::Vector3f(0,0,0);
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
		Relativty::ServerDriver::Log("CTRL COM: COM initialization failed. \"" + comportRAW + "\"\n");
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
		Relativty::ServerDriver::Log("CTRL| set to static Rotation mode.\n");
	}
	else {
		Relativty::ServerDriver::Log("CTRL| listening for Rotations mode.\n");
		this->retrieve_quaternion_isOn = true;
		this->retrieve_quaternion_thread_worker = std::thread(&Relativty::CTRLDriver::retrieve_device_quaternion_packet_threaded, this);
	}
	if (bIsTrackingInput) {
		this->retrieve_input_isOn = true;
		this->retrieve_input_thread_worker = std::thread(&Relativty::CTRLDriver::retrieve_client_input_packet_threaded, this);
	}
	if (this->start_tracking_server) {
		this->retrieve_vector_isOn = true;
		this->retrieve_vector_thread_worker = std::thread(&Relativty::CTRLDriver::retrieve_client_vector_packet_threaded, this);
		
		//this->startPythonTrackingClient_worker = std::thread(startPythonTrackingClient_threaded, this->PyPath);
	}
	while (this->serverNotReady) {
		// do nothing
	}
	Relativty::ServerDriver::Log("CTRL update_pose_threaded start \n");
	this->update_pose_thread_worker = std::thread(&Relativty::CTRLDriver::update_pose_threaded, this);
		
	return vr::VRInitError_None;
}

void Relativty::CTRLDriver::Deactivate() {
	this->retrieve_quaternion_isOn = false;
	this->retrieve_quaternion_thread_worker.join();
	if (this->bIsSerialComport) {
		CloseHandle(this->serialHandle);
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
		//Relativty::ServerDriver::Log("CTRL| CTRL| get vectorials '" + std::to_string(this->new_vector_avaiable) + "\n");
		vr::VRDriverInput()->UpdateBooleanComponent(HButtons[0], GetAsyncKeyState(0x5A) ? true : false, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(HButtons[1], GetAsyncKeyState(0x43) ? true : false, 0); // C
		vr::VRDriverInput()->UpdateBooleanComponent(HButtons[2], GetAsyncKeyState(0x56) ? true : false, 0); //v 
		vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], GetAsyncKeyState(0x42) ? true : false, 0); //B
		//vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], GetAsyncKeyState(0x42) ? true : false, 0);
		//vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], GetAsyncKeyState(0x4E) ? true : false, 0);

		if (this->new_input_avaiable) {
			//Relativty::ServerDriver::Log("CTRL| ARCH| SOCKET| get buttons\n");
			//0 is A etc
			//get real button 0's digital bound index :
			for (int i = 0; i < 6; i++)
			{
				int digitalIndex = this->binding.getButtonIndex(i);
				if(digitalIndex > -1)
					vr::VRDriverInput()->UpdateBooleanComponent(HButtons[i], this->input_bool[digitalIndex] == 1, 0);
			}

			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[2], this->input_bool[0] == 1 ? 1.0f : 0.0f, 0); //Trackpad z

			//vr::VRDriverInput()->UpdateBooleanComponent(HButtons[0], false, 0); //A				/input/application_menu/click
			//vr::VRDriverInput()->UpdateBooleanComponent(HButtons[1], false, 0); //B			/input/system/click
			//vr::VRDriverInput()->UpdateBooleanComponent(HButtons[2], false, 0); //trigger		/input/grip/click
			//vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], false, 0); //System		/input/trigger/click
			//vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], this->input_bool[0] == 1, 0); //Grip			/input/trackpad/click
			//vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], this->input_bool[0] == 1, 0); //				/input/trackpad/touch

			

			/*
			float scalarX = 0;
			if (bB && !bD) {
				scalarX = 1;
			}
			else if (bD && !bB) {
				scalarX = -1;
			}

			float scalarY = 0;
			if (bA && !bF) {
				scalarY = 1;
			}
			else if (bF && !bA) {
				scalarY = -1;
			}


			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[0], scalarX, 0); //Trackpad x
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[1], scalarY, 0); //Trackpad y
			*/
			//vr::VRDriverInput()->UpdateScalarComponent(HAnalog[2], this->binding.getButtonIndex(3) == 1 ? 1 : 0, 0); //Trigger
			Relativty::ServerDriver::Log("ARCH| CTRL| buttons updated\n");
			this->new_input_avaiable = false;
		}

		bool offsetChanged = false;
		if (GetAsyncKeyState(0x49)) { //I = Y+
			OY = OY + (1 / 10);
			offsetChanged = true;
		}
		if (GetAsyncKeyState(0x4B)) { //K = Y-
			OY = OY - (1 / 10);
			offsetChanged = true;
		}

		if (GetAsyncKeyState(0x4A)) { //J = X-
			OX = OX - (1 / 100);
			offsetChanged = true;
		}
		if (GetAsyncKeyState(0x4C)) { //L = X+
			OX = OX + (1 / 100);
			offsetChanged = true;
		}

		//if (GetAsyncKeyState(0x59)) { //Y = Z+
			//OZ = OZ + (1 / 1000);
			//offsetChanged = true;
		//}
		//if (GetAsyncKeyState(0x48)) { //H = Z-
			//OZ = OZ - (1 / 1000);
			//offsetChanged = true;
		//}
		if (offsetChanged) {
			Relativty::ServerDriver::Log("DriverFromHeadTranslation(" + std::to_string(OX)  + " , " + std::to_string(OY) + " , " + std::to_string(OZ) + " )\n");
			//m_Pose.qDriverFromHeadRotation.x = OX;
			//m_Pose.qDriverFromHeadRotation.y = OY;
			//m_Pose.qDriverFromHeadRotation.z = OZ;
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
		}

		if (this->new_quaternion_avaiable && this->new_vector_avaiable) {
			m_Pose.qRotation.w = this->quat[0];
			m_Pose.qRotation.x = this->quat[1];
			m_Pose.qRotation.y = this->quat[2];
			m_Pose.qRotation.z = this->quat[3];

			m_Pose.vecPosition[0] = this->vector_xyz[0];
			m_Pose.vecPosition[1] = this->vector_xyz[1];
			m_Pose.vecPosition[2] = this->vector_xyz[2];
			Relativty::ServerDriver::Log("CTRL| CTRL| Update POSE with vector and rotation\n");
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_quaternion_avaiable = false;
			this->new_vector_avaiable = false;

		}
		if (this->new_quaternion_avaiable) {
			m_Pose.qRotation.w = this->quat[0];
			m_Pose.qRotation.x = this->quat[1];
			m_Pose.qRotation.y = this->quat[2];
			m_Pose.qRotation.z = this->quat[3];
			//Relativty::ServerDriver::Log("ARCH|\tQuartonian Received\n");
			Relativty::ServerDriver::Log("CTRL| CTRL| Update POSE with rotation\n");
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_quaternion_avaiable = false;

		}
		if (this->new_vector_avaiable) {

			m_Pose.vecPosition[0] = this->vector_xyz[0];
			m_Pose.vecPosition[1] = this->vector_xyz[1];
			m_Pose.vecPosition[2] = this->vector_xyz[2];
			//Relativty::ServerDriver::Log("CTRL| CTRL| Update POSE with vector\n");
			//Relativty::ServerDriver::Log("ARCH|CTRL \tVector Received\n");
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
	Relativty::ServerDriver::Log("controller Thread rotation track: successfully started\n");
	while (this->retrieve_quaternion_isOn) {
		bool isAvailable = Relativty::ServerDriver::SOCKServer.isNewRotation('R');
		//Relativty::ServerDriver::Log("ARCH| CTRL| R| is available : " + isAvailable ? "YES\n" : "NO\n");
		if (!isAvailable)
			continue;
		SocketServer::rotationState state = Relativty::ServerDriver::SOCKServer.getRotationState('R');
		float* rotate = state.rotation;
		float rotation[4];

		rotation[0] = rotate[0] + 0.0f;
		rotation[1] = rotate[1] + 0.0f;
		rotation[2] = rotate[2] + 0.0f;
		rotation[3] = rotate[3] + 0.0f;

		Relativty::ServerDriver::Log("ARCH| CTRL| R| got out rW " + std::to_string(rotation[0]));
		Relativty::ServerDriver::Log("ARCH| CTRL| R| got out rX " + std::to_string(rotation[1]));
		Relativty::ServerDriver::Log("ARCH| CTRL| R| got out rY " + std::to_string(rotation[2]));
		Relativty::ServerDriver::Log("ARCH| CTRL| R| got out rZ " + std::to_string(rotation[3]));
		this->invertY = true;
		this->quat[0] = rotation[0] * (this->invertW ? -1.0f : 1.0f); //W
		this->quat[1] = rotation[1] * (this->invertX ? -1.0f : 1.0f); //X
		this->quat[2] = rotation[3] * (this->invertZ ? -1.0f : 1.0f); //Z
		this->quat[3] = rotation[2] * (this->invertY ? -1.0f : 1.0f); //Y

		//this->calibrate_quaternion();

		this->new_quaternion_avaiable = true;
	}
	Relativty::ServerDriver::Log("CTRL| Thread1: successfully stopped\n");
}

void Relativty::CTRLDriver::retrieve_client_input_packet_threaded() {
	Relativty::ServerDriver::Log("controller Thread Input GET; successfully started\n");
	while (this->retrieve_input_isOn) {
		if (bIsTrackingInput && Relativty::ServerDriver::SOCKServer.isNewInputs('R')) {
			SocketServer::inputState state = Relativty::ServerDriver::SOCKServer.getInputState('R');
			Relativty::ServerDriver::Log("ARCH| CTRL| is tracking input");
			int bA = state.button[0];
			int bB = state.button[1];
			int bC = state.button[2];
			int bD = state.button[3];
			int bE = state.button[4];
			int bF = state.button[5];
			int bG = state.button[6];
			int bH = state.button[7];
			Relativty::ServerDriver::Log("ARCH| CTRL| R| Button A " + std::to_string(bA));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| Button B " + std::to_string(bB));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| Button C " + std::to_string(bC));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| Button D " + std::to_string(bD));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| Button E " + std::to_string(bE));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| Button F " + std::to_string(bF));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| Button G " + std::to_string(bG));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| Button H " + std::to_string(bH));
			this->input_bool[0] = bA;
			this->input_bool[1] = bB;
			this->input_bool[2] = bC;

			/*
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &HButtons[0]); //f
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtons[1]); //h

			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &HButtons[2]); //g
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtons[3]); //l

			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &HButtons[4]); //j
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtons[5]); //k
			*/


			this->new_input_avaiable = true;
		}
	}
}

void Relativty::CTRLDriver::retrieve_client_vector_packet_threaded() {
	using Eigen::Vector3;
	float normalize_min[3]{ this->normalizeMinX, this->normalizeMinY, this->normalizeMinZ};
	float normalize_max[3]{ this->normalizeMaxX, this->normalizeMaxY, this->normalizeMaxZ};
	float scales_coordinate_meter[3]{ this->scalesCoordinateMeterX, this->scalesCoordinateMeterY, this->scalesCoordinateMeterZ};
	

	float coordinate_normalized[3];
	float coordinate_true_center[3];

	this->serverNotReady = false;

	Relativty::ServerDriver::Log("CTRL| Thread3: successfully started\n");
	while (this->retrieve_vector_isOn) {
		float offset_coordinate[3] = { this->offsetCoordinateX, this->offsetCoordinateY, this->offsetCoordinateZ };
		bool isAvailableR = Relativty::ServerDriver::SOCKServer.isNewCoordinates('R');
		Relativty::ServerDriver::Log("CTRL| is R available : " + std::to_string(isAvailableR) + "\n");


		if (GetAsyncKeyState(VK_UP))
		{
			if (GetAsyncKeyState(VK_RSHIFT)) {
				modX -= 0.05f;
			}
			else
			{
				modX += 0.05f;
			}
			
		}
		if (GetAsyncKeyState(VK_DOWN))
		{
			if (GetAsyncKeyState(VK_RSHIFT)) {
				modY -= 0.05f;
			}
			else
			{
				modY += 0.05f;
			}
			
		}
		if (GetAsyncKeyState(VK_LEFT))
		{
			if (GetAsyncKeyState(VK_RSHIFT)) {
				modZ -= 0.05f;
			}
			else
			{
				modZ += 0.05f;
			}
			

		}
		Relativty::ServerDriver::Log("Thread3: CTRL ModX is now '" + std::to_string(modX) + "'\n"); // ModX is now '-473.557983'
		Relativty::ServerDriver::Log("Thread3: CTRL ModY is now '" + std::to_string(modY) + "'\n"); // -423.870026
		Relativty::ServerDriver::Log("Thread3: CTRL ModZ is now '" + std::to_string(modZ) + "'\n"); //-807.526367
		bool isAvailable = isAvailableR;
		if (bIsStaticPosition) {
			float coordinate[3];
			coordinate[0] = 0.0f;
			coordinate[1] = 0.0f;
			coordinate[2] = 0.0f;
			Normalize(coordinate_normalized, coordinate, normalize_max, normalize_min, this->upperBound, this->lowerBound, scales_coordinate_meter, offset_coordinate);
			float rota[4]{ this->quat[0], this->quat[1], this->quat[2], this->quat[3] };
			Eigen::Vector3f coordinate_normalized_vector = Eigen::Vector3f{ coordinate_normalized[0],coordinate_normalized[1],coordinate_normalized[2] };

			Eigen::Vector3f forward_axis = Eigen::Vector3f{ 1,0,0 };

			Eigen::Vector3f true_center = getControllerTrueCenter(coordinate_normalized_vector, rota , forward_axis, 1);
			float coordinate_true_center[3] = { true_center[0],true_center[1], true_center[2] };
			this->vector_xyz[0] = coordinate_normalized[0];
			this->vector_xyz[1] = coordinate_normalized[1];
			this->vector_xyz[2] = coordinate_normalized[2];
			this->new_vector_avaiable = true;
		} else

		if (isAvailable) {
			Relativty::ServerDriver::Log("ARCH| CTRL| get state of 'R'\n");
			SocketServer::coordinateState state = Relativty::ServerDriver::SOCKServer.getCoordinateState('R');
			float coordinate[3];
			float* coordinateCapture = state.coordinate;
			coordinate[0] = coordinateCapture[0];
			coordinate[1] = coordinateCapture[1];
			coordinate[2] = coordinateCapture[2];
			

			if (this->resetCoordinateOrigin) {
				//reset so that origin is current coordinates :
				coordinateOrigin[0] = coordinate[0];
				coordinateOrigin[1] = coordinate[1];
				coordinateOrigin[2] = coordinate[2];
				this->resetCoordinateOrigin = false;
			}

			coordinate[0] = -(coordinateOrigin[0] - coordinate[0]);
			coordinate[1] = -(coordinateOrigin[1] - coordinate[1]);
			coordinate[2] = -(coordinateOrigin[2] - coordinate[2]);

			Relativty::ServerDriver::Log("ARCH| CTRL| R| got out X " + std::to_string(coordinate[0]));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| got out Y " + std::to_string(coordinate[1]));
			Relativty::ServerDriver::Log("ARCH| CTRL| R| got out Z " + std::to_string(coordinate[2]));
			Relativty::ServerDriver::Log("Thread3:CTR ModX is '" + std::to_string(modX) + "'\n");
			Relativty::ServerDriver::Log("Thread3:CTR ModY is '" + std::to_string(modY) + "'\n");
			Relativty::ServerDriver::Log("Thread3:CTR ModZ is '" + std::to_string(modZ) + "'\n");
			//if (applyCoordinates) {
			const float mod = fScaleBy;
			Relativty::ServerDriver::Log("ARCH| CTRL| SOCKET| scale by " + std::to_string(mod));
			coordinate[0] = coordinate[0] * mod;
			coordinate[1] = coordinate[1] * mod;
			coordinate[2] = coordinate[2] * mod;
				
			Relativty::ServerDriver::Log("ARCH| CTRL| SOCKET| normie");
			Normalize(coordinate_normalized, coordinate, normalize_max, normalize_min, this->upperBound, this->lowerBound, scales_coordinate_meter, offset_coordinate);
			float rota[4]{ this->quat[0], this->quat[1], this->quat[2], this->quat[3] };
			Eigen::Vector3f coordinate_normalized_vector = Eigen::Vector3f{ coordinate_normalized[0],-coordinate_normalized[1],-coordinate_normalized[2] };

			Eigen::Vector3f forward_axis = Eigen::Vector3f{ 0,0,1 };
			//this->OffsetDistance = 0.0f;
			if (GetAsyncKeyState(0x57)) //W key
				this->OffsetDistance = this->OffsetDistance + 0.01f;

			if (GetAsyncKeyState(0x45)) //# key
				this->OffsetDistance = this->OffsetDistance - 0.01f;

			if (GetAsyncKeyState(0x51) && GetAsyncKeyState(0xA4)) //Q? key
				this->resetCoordinateOrigin = true;

			Relativty::ServerDriver::Log("ARCH| CTRL| OFFSET DISANCE IS " + std::to_string(OffsetDistance) + "\n",true);


			Eigen::Vector3f true_center = getControllerTrueCenter(coordinate_normalized_vector, rota, forward_axis, OffsetDistance);
			float coordinate_true_center[3] = { true_center[0] + modX,true_center[1] + modY, true_center[2] + modZ };

			//float rota[4]{ this->quat[0], this->quat[1], this->quat[2], this->quat[3] };
			//Relativty::ServerDriver::Log("Thread3: use Axis Angle '" + this->axisAngleManager.getAxisAngleName() + "' & distance '" + std::to_string(LEDIMUDistance) + "'\n");
			
			//Eigen::Vector3f coordinate_true_center = Eigen::Vector3f{ coordinate_normalized[1],coordinate_normalized[2], coordinate_normalized[0] };

			//Eigen::Vector3f true_center = getControllerTrueCenter(coordinate_true_center, rota, this->axisAngleManager.getAxisAngle(), this->LEDIMUDistance);
				
			this->vector_xyz[0] = coordinate_true_center[0];
			this->vector_xyz[1] = coordinate_true_center[1];
			this->vector_xyz[2] = coordinate_true_center[2];

			Relativty::ServerDriver::Log("RED CONTROLLER POSE POSITION RESOLVED TO (" + std::to_string(vector_xyz[0]) + ", " + std::to_string(vector_xyz[1]) + ", " + std::to_string(vector_xyz[2]) + ")\n");


			this->new_vector_avaiable = true;
		}
	}
	Relativty::ServerDriver::Log("CTRL| Thread3: successfully stopped\n");

}

Relativty::CTRLDriver::CTRLDriver(std::string myserial):RelativtyDevice(myserial, "akira_") {
	// keys for use with the settings API
	static const char* const Relativty_ctrl_section = "Relativty_ctrl";


	// openvr api stuff
	m_sRenderModelPath = "C:\\Users\\Arch\\Documents\\SteamVR\\Relativty\\Relativty\\Relativty_Driver\\Relativty\\resources\\rendermodels\\generic_ctrl";
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

	float axisAngleX = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "axisAngleX");
	float axisAngleY = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "axisAngleY");
	float axisAngleZ = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "axisAngleZ");

	float LEDIMUDx = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "LEDIMUDistance");

	this->axisAngle[0] = axisAngleX;
	this->axisAngle[0] = axisAngleY;
	this->axisAngle[0] = axisAngleZ;

	this->LEDIMUDistance = LEDIMUDx;

	this->fScaleBy = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "scaleBy");

	this->modX = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "modX");
	this->modY = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "modY");
	this->modZ = vr::VRSettings()->GetFloat(Relativty_ctrl_section, "modZ");

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
	this->bIsTrackingInput = vr::VRSettings()->GetBool(Relativty_ctrl_section, "willTrackInput");

	for (int i = 0; i < 6; i++)
	{
		std::string strrr = "B" + std::to_string(i);
		this->binding.buttons[i] = vr::VRSettings()->GetInt32(Relativty_ctrl_section, strrr.c_str());
	}
	

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
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtons[1]); //h //Button C

	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &HButtons[2]); //g //V
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &HButtons[3]); //l //B

	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &HButtons[4]); //j
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtons[5]); //k

	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/thumbstick/x", &HAnalog[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/thumbstick/y", &HAnalog[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalog[2], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
}

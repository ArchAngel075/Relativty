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

#pragma once
#include <thread>
#include <atomic>
#include <WinSock2.h>
#include "hidapi/hidapi.h"
#include "openvr_driver.h"
#include "Relativty_components.h"
#include "Relativty_base_device.h"
#include <codecvt>


namespace Relativty {
	class CTRLDriver : public RelativtyDevice<false>
	{
	public:
		CTRLDriver(std::string myserial);
		~CTRLDriver() = default;

		void frameUpdate();
		inline void setProperties();

		void closeCom();
		void openCom();

		// Inherited from RelativtyDevice, to be overridden
		virtual vr::EVRInitError Activate(uint32_t unObjectId);
		virtual void Deactivate();

	private:
		vr::VRInputComponentHandle_t HButtons[6], HAnalog[3];
		int32_t m_iPid;
		int32_t m_iVid;
		std::wstring comport;
		std::string comportRAW;
		bool bIsSerialComport;
		bool bIsStaticPosition;
		bool bIsStaticRotation;

		bool m_bIMUpktIsDMP;

		float SecondsFromVsyncToPhotons;
		float DisplayFrequency;
		float IPD;
		float HeadToEyeDepth;

		vr::DriverPose_t lastPose = {0};
		hid_device* handle;
		HANDLE serialHandle;

		std::atomic<float> quat[4];
		std::atomic<bool> retrieve_quaternion_isOn = false;
		std::atomic<bool> new_quaternion_avaiable = false;

		std::atomic<float> qconj[4] = {1, 0, 0, 0};
		void calibrate_quaternion();

		std::thread retrieve_quaternion_thread_worker;
		void retrieve_device_quaternion_packet_threaded();

		std::atomic<float> vector_xyz[3];
		std::atomic<bool> retrieve_vector_isOn = false;
		std::atomic<bool> new_vector_avaiable = false;
		bool start_tracking_server = true;
		SOCKET sock, sock_receive;
		float upperBound;
		float lowerBound;

		float normalizeMinX;
		float normalizeMinY;
		float normalizeMinZ;

		float normalizeMaxX;
		float normalizeMaxY;
		float normalizeMaxZ;

		float scalesCoordinateMeterX;
		float scalesCoordinateMeterY;
		float scalesCoordinateMeterZ;

		float offsetCoordinateX;
		float offsetCoordinateY;
		float offsetCoordinateZ;

		std::atomic<bool> serverNotReady = true;
		std::thread retrieve_vector_thread_worker;
		void retrieve_client_vector_packet_threaded();

		std::thread update_pose_thread_worker;
		void update_pose_threaded();

		std::string PyPath;
		std::thread startPythonTrackingClient_worker;
	};
}
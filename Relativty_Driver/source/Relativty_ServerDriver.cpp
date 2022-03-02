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

#include "openvr_driver.h"

#include "driverlog.h"

#include "Relativty_ServerDriver.hpp"
#include "Relativty_HMDDriver.hpp"
#include "Relativty_CTRLDriver.hpp"

vr::EVRInitError Relativty::ServerDriver::Init(vr::IVRDriverContext* DriverContext) {

	vr::EVRInitError eError = vr::InitServerDriverContext(DriverContext);
		if (eError != vr::VRInitError_None) {
			return eError;
	}
	#ifdef DRIVERLOG_H
	InitDriverLog(vr::VRDriverLog());
	DriverLog("Relativty driver version 0.1.1 | Modified v2"); // report driver version
	DriverLog("Thread1: hid quaternion packet listener loop");
	DriverLog("Thread2: update driver pose loop");
	DriverLog("Thread3: receive positional data from python loop");
	#endif

	this->Log("Relativty Init successful.\n");
	this->Log("Relativty Start Socket Server.\n");
	Relativty::ServerDriver::SOCKServer.open();
	this->Log("Relativty Socket Server Started.\n");

	this->Log("Relativty boot HMD device.\n");
	this->HMDDriver = new Relativty::HMDDriver("zero");
	vr::VRServerDriverHost()->TrackedDeviceAdded(HMDDriver->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_HMD, this->HMDDriver);


	//this->Log("Relativty boot CTRL device.\n");
	//this->CTRLDriver = new Relativty::CTRLDriver("one");
	//vr::VRServerDriverHost()->TrackedDeviceAdded(CTRLDriver->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, this->CTRLDriver);

	
	//vr::VRServerDriverHost()->TrackedDeviceAdded
	// GetSerialNumber() is there for a reason!

	return vr::VRInitError_None;
}

void Relativty::ServerDriver::Cleanup() {
	if (globalExceptionPtr)
	{
		try
		{
			std::rethrow_exception(globalExceptionPtr);
		}
		catch (const std::exception& ex)
		{
			std::string err = std::string(ex.what());
			this->Log("Thread failed with exception. " + err + "\n");
		}
	}
	Relativty::SocketServer sss;
	sss.close();

	delete this->HMDDriver;
	this->HMDDriver = NULL;

	delete this->CTRLDriver;
	this->CTRLDriver = NULL;

	#ifdef DRIVERLOG_H
	CleanupDriverLog();
	#endif

	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* Relativty::ServerDriver::GetInterfaceVersions() {
	return vr::k_InterfaceVersions;
}

void Relativty::ServerDriver::RunFrame() {} // if ur not using it don't populate it with garbage!

bool Relativty::ServerDriver::ShouldBlockStandbyMode() {
	return false;
}

void Relativty::ServerDriver::EnterStandby() {

}

void Relativty::ServerDriver::LeaveStandby() {

}

void Relativty::ServerDriver::Log(std::string log) {
	vr::VRDriverLog()->Log(log.c_str());
}

void Relativty::ServerDriver::worker_threader()
{
	while (true) {
		if (globalExceptionPtr)
		{
			this->Log("Thread failed with exception....\n");
			try
			{
				std::rethrow_exception(globalExceptionPtr);
			}
			catch (const std::exception& ex)
			{
				std::string err = std::string(ex.what());
				this->Log("Thread failed with exception. " + err + "\n");
			}
		}
	}
}

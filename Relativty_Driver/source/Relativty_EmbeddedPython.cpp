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
#pragma comment(lib, "User32.lib")
#pragma comment (lib, "Setupapi.lib")
#pragma comment (lib, "python38.lib")

#include <iostream>
#include <filesystem>
#include <string>
#include "Python.h"
#include "Relativty_EmbeddedPython.h"
#include "Relativty_ServerDriver.hpp"

namespace fs = std::filesystem;

void startPythonTrackingClient_threaded(std::string PyPath) {
	Relativty::ServerDriver::Log("Thread4: ARCH| python init\n");
	std::string fileName = PyPath + "/main.py";
	//FILE* fp;
	//fp = fopen(fileName.c_str(), "rb");

	std::string singleQuote = "\'";
	PyPath = "PyPATH = " + singleQuote + PyPath + singleQuote;
	Relativty::ServerDriver::Log("Thread4: ARCH| Client at '" + PyPath + "'\n");
	/*
	Py_Initialize();
	PyRun_SimpleString("import sys");
	Relativty::ServerDriver::Log("Thread4: ARCH| python path setup\n");
	PyRun_SimpleString("sys.path.append('C:\\Users\\Arch\\Documents\\SteamVR\\pyRGBDetect')");
	Relativty::ServerDriver::Log("Thread4: ARCH| script running\n");
	//PyRun_SimpleString(PyPath.c_str());
	Relativty::ServerDriver::Log("Thread4: starting Client.py \n");
	int code = PyRun_AnyFileExFlags(fp, "C:\\Users\\Arch\\Documents\\SteamVR\\pyRGBDetect\\hsv_v2.py", 0, NULL);
	*/
	char filename[] = "C:\\Users\\Arch\\Documents\\SteamVR\\pyRGBDetect\\hsv_v2.py";
	FILE* fp;
	Py_Initialize();

	fp = _Py_fopen(filename, "r");
	PyRun_SimpleFile(fp, filename);

	Py_Finalize();
	Relativty::ServerDriver::Log("Thread4: ARCH| exit '\n");
	Py_Finalize();
}
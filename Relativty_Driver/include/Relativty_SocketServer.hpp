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
	class SocketServer
	{
	private:
		
		bool run_thread = true;
		struct positionPacket {
			float X;
			float Y;
			float S;
			int colID;
			int CamID;
		};
		struct inputPacket {
			int colID;
			float B1;
			float B2;
			float B3;
			float B4;
			float B5;
			float B6;
			float B7;
			float B8;
		};

		char rgbToChar(float R, float G, float B);
		SOCKET sock;
		SOCKET sock_receive[3];
		std::atomic<bool> bIsStaticPosition = false;

		std::atomic<float> coordinate[3][3]{ { 1, 1, 1 }, { 1, 1, 1 }, { 1, 1, 1 } };

		std::atomic<int> colID[3] = { 0,1,2 };
		std::atomic<char> colChar[3] = { 'R','G','B' };

		std::atomic<float> buttons[3][8] = { { 0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,0,0,0 }, { 0,0,0,0,0,0,0,0 } };

		std::atomic<bool> freshState[3] = { false, false, false };
	public:
		std::thread cycle_receive_parse_packets_thread_worker;
		void cycle();
		std::atomic<bool> readyness = false;
		struct deviceState {
			float* coordinate;
			float* button;
		};
		

		void open();
		

		void parsePacket(const char* packet);

		positionPacket parsePositionPacket(const char* packet);
		void processPositionPacket(positionPacket p);

		inputPacket parseInputPacket(const char* packet);
		void processInputPacket(inputPacket packet);

		int colorToIdentity(float R, float G, float B);

		int charToIdentity(char ch);

		void close();

		bool isAvailable(int id);
		bool isAvailable(char id);

		deviceState getState(int id);
		deviceState getState(char id);
	};

}
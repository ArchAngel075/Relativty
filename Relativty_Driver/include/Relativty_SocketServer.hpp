#pragma once
#include <thread>
#include <atomic>
#include <WinSock2.h>
#include "hidapi/hidapi.h"
#include "openvr_driver.h"
#include "Relativty_components.h"
#include "Relativty_base_device.h"

#include <codecvt>
#include <vector>

namespace Relativty {
	class SocketServer
	{
	public:
		int devices;
		bool run_thread = true;
		struct packetPart {
			byte a;
			byte b;
			byte c;
			byte d;
			char getChar() {
				return a;
			};
			int getInt() {
				//calc b's contribution :
				//for each a b c d do the rules :
				int baddr = (b * 256);
				if (a < 0) {
					return a + 256 + baddr;
				}
				else {
					return a + baddr;
				}
			};
			bool isChar(char ch) {
				return (char)a == ch && b == 0 && c == 0 && d == 0;
			};
		};
		struct sockPacket {
			private:
				Relativty::SocketServer::packetPart parts[256];
				int size = -1;
			public:
				void compose(Relativty::SocketServer::packetPart parts[256], int iFrom, int iTo) {
					for (int i = iFrom; i < iTo; i++)
					{
						addPart(parts[i]);
					}
				};
				void addPart(Relativty::SocketServer::packetPart p) {
					size++;
					parts[size] = p;
				};
				char getChar(int i) {
					return get(i).getChar();
				};
				int getInt(int i) {
					return get(i).getInt();
				};
				Relativty::SocketServer::packetPart get(int i) {
					return parts[i];
				};
		};
		struct rotationPacket {
			int colID;
			int W;
			int X;
			int Y;
			int Z;
		};
		struct positionPacket {
			float X;
			float Y;
			float S;
			int colID;
			int CamID;
		};
		struct inputPacket {
			int colID;
			int B1;
			int B2;
			int B3;
			int B4;
			int B5;
			int B6;
			int B7;
			int B8;
		};

		char rgbToChar(float R, float G, float B);
		SOCKET sock;
		SOCKET sock_receive[3];
		bool sock_state[3];
		std::atomic<bool> bIsStaticPosition = false;

		std::atomic<float> coordinate[3][3]{ { 0, 0, 0 } ,{ 0, 0, 0 }, { 0, 0, 0 } };
		std::atomic<float> rotation[3][4]{ { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
		std::atomic<float> coordinateOrigin[3][3]{ { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		//std::vector<std::vector<Eigen::Vector3f>> CameraPointsPerFrame;

		std::atomic<int> colID[3] = { 0,1,2 };
		std::atomic<char> colChar[3] = { 'R','G','B' };

		std::atomic<int> buttons[3][8] = { { 0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,0,0,0 }, { 0,0,0,0,0,0,0,0 } };

		std::atomic<bool> hasNewCoordinatesState[3] = { false, false, false };
		std::atomic<bool> hasNewRotationState[3] = { false, false, false };
		std::atomic<bool> hasNewInputState[3] = { false, false, false };

		std::thread cycle_receive_parse_packets_thread_worker;
		void cycle();
		std::atomic<bool> readyness = false;

		struct coordinateState {
			float* coordinate;
		};

		struct rotationState {
			float* rotation;
		};

		struct inputState {
			float* button;
		};

		bool _USE_V4_PROTOCOL_ = false;

		void open();

		void precv(int i);
		void precv(char c[], int a, bool r = false);

		void parsePacket(sockPacket packet);

		positionPacket parsePositionPacket(sockPacket packet);
		void processPositionPacket(positionPacket p);

		inputPacket parseInputPacket(sockPacket packet);
		void processInputPacket(inputPacket packet);

		rotationPacket parseRotationPacket(sockPacket packet);
		void processRotationPacket(rotationPacket packet);
		void processRotationPacket(std::string packet);

		//v4 protocol :
		void parsePositionPacketFrame(sockPacket packet);

		int colorToIdentity(float R, float G, float B);

		int charToIdentity(char ch);

		char identityToChar(int i);

		void close();

		bool isNewCoordinates(int id);
		bool isNewCoordinates(char id);

		bool isNewRotation(int id);
		bool isNewRotation(char id);

		bool isNewInputs(int id);
		bool isNewInputs(char id);

		coordinateState getCoordinateState(int id);
		coordinateState getCoordinateState(char id);

		rotationState getRotationState(int id);
		rotationState getRotationState(char id);

		inputState getInputState(int id);
		inputState getInputState(char id);
	};

}
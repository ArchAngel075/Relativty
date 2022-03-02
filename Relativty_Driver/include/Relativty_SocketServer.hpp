#pragma once
namespace Relativty {
	class Relativty_SocketServer
	{
	private:
		std::thread cycle_receive_parse_packets_thread_worker;
		struct positionPacket {
			float X;
			float Y;
			float S;
			int colID;
			float CamID;
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
		int fd;
		SOCKET sock;
		SOCKET sock_receive[3];
		bool bIsStaticPosition = false;

		float coordinate[3][3]{ { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

		int colID[3] = { 0,1,2 };
		char colChar[3] = { 'R','G','B' };

		float buttons[3][8] = { { 0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,0,0,0 }, { 0,0,0,0,0,0,0,0 } };
	public:
		struct deviceState {
			float* coordinate;
			float* button;
		};
		

		void open();
		void cycle();

		void parsePacket(const char* packet);

		positionPacket parsePositionPacket(const char* packet);
		void processPositionPacket(positionPacket p);

		inputPacket parseInputPacket(const char* packet);
		void processInputPacket(inputPacket packet);

		int colorToIdentity(float R, float G, float B);

		int charToIdentity(char ch);

		void close();

		deviceState getState(int id);
		deviceState getState(char id);

		char* lastPacket;
	};

}
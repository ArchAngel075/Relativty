#include "Relativty_SocketServer.hpp"
#include <WinSock2.h>
#include <io.h>
#include <Relativty_ServerDriver.hpp>

/*
	Create socket server with 3 potential connections (RGB based)
	When a packet comes in 
		identify the device (R or G or B)
		identify the camera (1 or 2)
		update the local state of the device

	getState('R'|'G'|'B') returns the current latest state for that color,
	//the current binding is R&G = None , B = HMD
	//All devices are allocated 8 buttons at the moment. including HMD.
*/

//create and open the socket server and listen for conections.
void Relativty::Relativty_SocketServer::open()
{
	WSADATA wsaData;
	struct sockaddr_in server, client;
	int addressLen;

	Relativty::ServerDriver::Log("Thread3: Initialising Socket.\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		Relativty::ServerDriver::Log("Thread3: Failed. Error Code: " + WSAGetLastError());
		return;
	}
	Relativty::ServerDriver::Log("Thread3: Socket successfully initialised.\n");

	if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
		Relativty::ServerDriver::Log("Thread3: could not create socket: " + WSAGetLastError());
	Relativty::ServerDriver::Log("Thread3: Socket created.\n");

	server.sin_family = AF_INET;
	server.sin_port = htons(50000);
	server.sin_addr.s_addr = INADDR_ANY;

	if (bind(this->sock, (struct sockaddr*) & server, sizeof(server)) == SOCKET_ERROR)
		Relativty::ServerDriver::Log("Thread3: Bind failed with error code: " + WSAGetLastError());
	Relativty::ServerDriver::Log("Thread3: Bind done \n");
	//Relativty::ServerDriver::Log("ARCH| HMD| SOCK|A track? " + std::to_string(bIsTrackingA) + "\n");
	//Relativty::ServerDriver::Log("ARCH| HMD| SOCK|B track? " + std::to_string(bIsTrackingB) + "\n");

	listen(this->sock, 3);

	if (!bIsStaticPosition) {
		Relativty::ServerDriver::Log("Thread3: Waiting for incoming connections...\n");
		addressLen = sizeof(struct sockaddr_in);
		this->sock_receive[0] = accept(this->sock, (struct sockaddr*) & client, &addressLen);
		if (this->sock_receive[0] == INVALID_SOCKET)
			Relativty::ServerDriver::Log("Thread3: accept failed with error code: " + WSAGetLastError());
		Relativty::ServerDriver::Log("Thread3: Connection A accepted");

		this->sock_receive[1] = accept(this->sock, (struct sockaddr*) & client, &addressLen);
		if (this->sock_receive[1] == INVALID_SOCKET)
			Relativty::ServerDriver::Log("Thread3: accept failed with error code: " + WSAGetLastError());
		Relativty::ServerDriver::Log("Thread3: Connection B accepted");

		this->sock_receive[2] = accept(this->sock, (struct sockaddr*) & client, &addressLen);
		if (this->sock_receive[2] == INVALID_SOCKET)
			Relativty::ServerDriver::Log("Thread3: accept failed with error code: " + WSAGetLastError());
		Relativty::ServerDriver::Log("Thread3: Connection C accepted");
	}

	this->cycle_receive_parse_packets_thread_worker = std::thread(&Relativty::Relativty_SocketServer::cycle, this);

	Relativty::ServerDriver::Log("Thread3: successfully started\n");
}

void Relativty::Relativty_SocketServer::cycle()
{
	while (true) {
		for (size_t i = 0; i < 3; i++)
		{
			int receiveBufferLen = 128;
			char receiveBuffer[128];
			//Relativty::ServerDriver::Log("ARCH| HMD| SOCK|A start\n");
			int resultReceiveLen = recv(this->sock_receive[i], receiveBuffer, receiveBufferLen, NULL);
			if (resultReceiveLen == -1) {
				fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
				Relativty::ServerDriver::Log("ARCH| HMD| SOCK| recv error [" + std::to_string(errno) + "].");
				continue;
			}
			if (resultReceiveLen > 0) {
				std::string receiveString = std::string(receiveBuffer);
				Relativty::ServerDriver::Log("ARCH| SOCKET| RAW Received '" + receiveString + "'\n");
				size_t opens = receiveString.find("{");
				if (opens != 0) {
					Relativty::ServerDriver::Log("ARCH| SOCKET| incomplete packet. Discarding'\n");
					continue;
				}
				size_t close = receiveString.find("}") + 1;
				receiveString.resize(close);
				Relativty::ServerDriver::Log("ARCH| SOCKET| packet length : " + std::to_string(receiveString.length()) + "\n");
				if (receiveString.length() < 7) {
					Relativty::ServerDriver::Log("ARCH| SOCKET| Bad packet length. Discarding'\n");
					continue;
				}
				// declaring character array
				const char* cstr = receiveString.c_str();
				parsePacket(cstr);
			}
		}
	}
}

void Relativty::Relativty_SocketServer::parsePacket(const char* packet) {
	//the proposed protocol for any packet follows :
	/*
		[Type=P][ID] + [X][Y][S][R][G][B]
		OR
		[Type=I][ID] + [B1][B2][B3][B4][B5][B6][B7][B8]

	*/
	char packetType = packet[0];
	switch (packetType)
	{
	case 'P':
		positionPacket pposition = parsePositionPacket(packet);
		processPositionPacket(pposition);
		break;
	case 'I':
		inputPacket pinput = parseInputPacket(packet);
		processInputPacket(pinput);
		break;
	default:
		Relativty::ServerDriver::Log("ARCH| received invalid packet type '" + std::to_string(packetType) + "'. expected 'P' or 'I' .\n");
		exit(0);
		break;
	}
}

void Relativty::Relativty_SocketServer::processPositionPacket(positionPacket packet) {
	//process the packet into coordinates.
	const int i = packet.colID;
	if (packet.CamID == 1) {//the front side camera
		coordinate[i][0] = packet.X * -1;
		coordinate[i][1] = coordinate[i][1];
		coordinate[i][2] = packet.Y * -1;
	}
	if (packet.CamID == 2) {//the right side camera
		coordinate[i][0] = coordinate[i][0];
		coordinate[i][1] = packet.X;
		coordinate[i][2] = coordinate[i][2];
	}
}

Relativty::Relativty_SocketServer::positionPacket Relativty::Relativty_SocketServer::parsePositionPacket(const char* packet) {
	//the proposed protocol for position packet follows :
	/*
			0	 1		2  3  4  5  6  7
		[Type=P][ID] + [X][Y][S][R][G][B]
	*/
	
	float packetType = int(packet[0]);
	int camId = int(packet[1]);

	float xf = float(packet[2]);
	float yf = float(packet[3]);

	float sf = float(packet[4]);

	float rf = float(packet[5]);
	float gf = float(packet[6]);
	float bf = float(packet[7]);

	int colID = colorToIdentity(rf, gf, bf);

	return positionPacket{ xf,yf,sf,colID, camId };
}

void Relativty::Relativty_SocketServer::processInputPacket(Relativty::Relativty_SocketServer::inputPacket packet) {
	//process the packet into coordinates.
	const int i = packet.colID;
	buttons[i][0] = float(packet.B1);
	buttons[i][1] = float(packet.B2);
	buttons[i][2] = float(packet.B3);
	buttons[i][3] = float(packet.B4);
	buttons[i][4] = float(packet.B5);
	buttons[i][5] = float(packet.B6);
	buttons[i][6] = float(packet.B7);
	buttons[i][7] = float(packet.B8);
}

Relativty::Relativty_SocketServer::inputPacket Relativty::Relativty_SocketServer::parseInputPacket(const char* packet) {
	//the proposed protocol for position packet follows :
	/*
			0	 1		2	3	4	5	6	7	8	9
		[Type=P][ID] + [B1][B2][B3][B4][B5][B6][B7][B8]
	*/

	int colID = int(packet[1]);

	return inputPacket{ int(packet[1]), float(packet[2]),float(packet[3]),float(packet[4]),
			float(packet[5]) ,float(packet[6]) ,float(packet[7]) ,float(packet[8]),
			float(packet[9]) };
}

char Relativty::Relativty_SocketServer::rgbToChar(float R, float G, float B) {
	return colChar[colorToIdentity(R, G, B)];
}

int Relativty::Relativty_SocketServer::colorToIdentity(float R, float G, float B)
{
	return (R >= G && R >= B) ? 0 : ((G >= R && G >= B) ? 1 : ((B >= R && B >= G) ? 2 : -1) );
}

int Relativty::Relativty_SocketServer::charToIdentity(char ch) {
	return ch == 'R' ? 0 : (ch == 'G' ? 1 : (ch == 'B' ? 2 : -1)) ;
}

//Close the socket server, informing all connections.
void Relativty::Relativty_SocketServer::close()
{
	closesocket(sock);
	this->cycle_receive_parse_packets_thread_worker.join();
	WSACleanup();
}

Relativty::Relativty_SocketServer::deviceState Relativty::Relativty_SocketServer::getState(int id) {
	float* coord = coordinate[id];
	float* button = buttons[id];
	return deviceState{coord, button};
}

Relativty::Relativty_SocketServer::deviceState Relativty::Relativty_SocketServer::getState(char id) {
	return getState(charToIdentity(id));
}

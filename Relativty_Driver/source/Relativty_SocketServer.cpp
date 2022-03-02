#define _CRT_SECURE_NO_WARNINGS
#include "Relativty_SocketServer.hpp"
#include "Relativty_ServerDriver.hpp"
#include <WinSock2.h>
#include <string>
#include <io.h>

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
void Relativty::SocketServer::open()
{
	WSADATA wsaData;
	struct sockaddr_in server, client;
	int addressLen;

	Relativty::ServerDriver::Log("Initialising Socket Server.\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		Relativty::ServerDriver::Log("SocketServer: Failed. Error Code: " + WSAGetLastError());
		return;
	}
	Relativty::ServerDriver::Log("SocketServer: Socket successfully initialised.\n");

	if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
		Relativty::ServerDriver::Log("SocketServer: could not create socket: " + WSAGetLastError());
	Relativty::ServerDriver::Log("SocketServer: Socket created.\n");

	server.sin_family = AF_INET;
	server.sin_port = htons(50000);
	server.sin_addr.s_addr = INADDR_ANY;

	if (bind(this->sock, (struct sockaddr*) & server, sizeof(server)) == SOCKET_ERROR)
		Relativty::ServerDriver::Log("SocketServer: Bind failed with error code: " + WSAGetLastError());
	Relativty::ServerDriver::Log("SocketServer: Bind done \n");

	listen(this->sock, 2);

	if (!bIsStaticPosition) {
		Relativty::ServerDriver::Log("SocketServer: Waiting for incoming connections...\n");
		addressLen = sizeof(struct sockaddr_in);
		this->sock_receive[0] = accept(this->sock, (struct sockaddr*) & client, &addressLen);
		if (this->sock_receive[0] == INVALID_SOCKET)
			Relativty::ServerDriver::Log("SocketServer: accept failed with error code: " + WSAGetLastError());
		Relativty::ServerDriver::Log("SocketServer: Connection A accepted");

		this->sock_receive[1] = accept(this->sock, (struct sockaddr*) & client, &addressLen);
		if (this->sock_receive[1] == INVALID_SOCKET)
			Relativty::ServerDriver::Log("SocketServer: accept failed with error code: " + WSAGetLastError());
		Relativty::ServerDriver::Log("SocketServer: Connection B accepted");

		//this->sock_receive[2] = accept(this->sock, (struct sockaddr*) & client, &addressLen);
		//if (this->sock_receive[2] == INVALID_SOCKET)
			//Relativty::ServerDriver::Log("SocketServer: accept failed with error code: " + WSAGetLastError());
		//Relativty::ServerDriver::Log("SocketServer: Connection C accepted");
	}

	Relativty::ServerDriver::Log("SocketServer: Start packet parser thread 3.A:");
	this->cycle_receive_parse_packets_thread_worker = std::thread(&Relativty::SocketServer::cycle, this);
	Relativty::ServerDriver::Log("SocketServer: packet parser thread thread started");

	Relativty::ServerDriver::Log("SocketServer: packet parser thread 3.A \n");

	//this->cycle_receive_parse_packets_thread_worker.join();
	this->readyness = true;
}

void Relativty::SocketServer::cycle()
{
	for (int i = 0; i < 3;i++) {
		coordinate[i][0] = 0;
		coordinate[i][1] = 0;
		coordinate[i][2] = 0;
	}

	Relativty::ServerDriver::Log("ARCH| HMD| SOCKET| CYCLE START!!\n");
	while (this->run_thread) {
		Relativty::ServerDriver::Log("ARCH| HMD| SOCKET| CYCLE\n");
		for (int i = 0; i < 2; i++)
		{
			int receiveBufferLen = 128;
			char receiveBuffer[128];
			Relativty::ServerDriver::Log("ARCH| HMD| SOCKET| check #" + std::to_string(i) + "~ \n");
			int resultReceiveLen = recv(this->sock_receive[i], receiveBuffer, receiveBufferLen, NULL);
			if (resultReceiveLen == -1) {
				//fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
				Relativty::ServerDriver::Log("ARCH| HMD| SOCKET| recv error [" + std::to_string(errno) + "].");
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
				Relativty::ServerDriver::Log("ARCH| SOCKET| Parse and process packet.'\n");
				parsePacket(cstr);
			}
		}
	}
	Relativty::ServerDriver::Log("ARCH| HMD| SOCKET| thread ended!!\n");
}

void Relativty::SocketServer::parsePacket(const char* packet) {
	//the proposed protocol for any packet follows :
	/*
		[Type=P][ID] + [X][Y][S][R][G][B]
		OR
		[Type=I][ID] + [B1][B2][B3][B4][B5][B6][B7][B8]

	*/
	char packetType = packet[1];
	std::string strr = std::to_string(packetType);
	Relativty::ServerDriver::Log("ARCH| SOCKET| packet type value '" + strr + "'\n");
	switch (packetType)
	{
	case 'P':
		Relativty::ServerDriver::Log("ARCH| SOCKET| packet type 'Position'\n");
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

void Relativty::SocketServer::processPositionPacket(positionPacket packet) {
	//process the packet into coordinates.
	Relativty::ServerDriver::Log("ARCH| SOCKET| Process position packet start\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| packet came from camera #" + std::to_string(packet.CamID) + "\n");
	if (packet.CamID == 1) {//the front side camera
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates on " + std::to_string(packet.colID) + "\n");

		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates X " + std::to_string(packet.X * -1) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates Y " + std::to_string(coordinate[packet.colID][1]) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates Z " + std::to_string(packet.Y * -1) + "\n");
		coordinate[packet.colID][0] = packet.X*-1;
		coordinate[packet.colID][1] = float(coordinate[packet.colID][1]);
		coordinate[packet.colID][2] = packet.Y*-1;

		Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set X " + std::to_string(coordinate[packet.colID][0]) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set Y " + std::to_string(coordinate[packet.colID][1]) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set Z " + std::to_string(coordinate[packet.colID][2]) + "\n");
	}
	if (packet.CamID == 2) {//the right side camera
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates on " + std::to_string(packet.colID) + "\n");

		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates X " + std::to_string(coordinate[packet.colID][0]) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates Y " + std::to_string(packet.X) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates Z " + std::to_string(coordinate[packet.colID][2]) + "\n");

		coordinate[packet.colID][0] = float(coordinate[packet.colID][0]);
		coordinate[packet.colID][1] = packet.X;
		coordinate[packet.colID][2] = packet.Y;

		Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set X " + std::to_string(coordinate[packet.colID][0]) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set Y " + std::to_string(coordinate[packet.colID][1]) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set Z " + std::to_string(coordinate[packet.colID][2]) + "\n");
	}
	freshState[packet.colID] = true;
	Relativty::ServerDriver::Log("ARCH| SOCKET| Process position packet end\n");
}

Relativty::SocketServer::positionPacket Relativty::SocketServer::parsePositionPacket(const char* packet) {
	//the proposed protocol for position packet follows :
	/*
			0	 1		2  3  4  5  6  7
		[Type=P][ID] + [X][Y][S][R][G][B]
	*/

	Relativty::ServerDriver::Log("ARCH| SOCKET| begin parse position packet\n");
	
	float packetType = int(packet[1]);
	Relativty::ServerDriver::Log("ARCH| SOCKET| parse position packet ID\n");
	int camId = int(packet[2]);


	Relativty::ServerDriver::Log("ARCH| SOCKET| parse position packet XY\n");
	float xf = float(packet[3]);
	float yf = float(packet[4]);

	Relativty::ServerDriver::Log("ARCH| SOCKET| parse position packet S\n");
	float sf = float(packet[5]);

	Relativty::ServerDriver::Log("ARCH| SOCKET| parse position packet RGB\n");
	float rf = float(packet[6]);
	float gf = float(packet[7]);
	float bf = float(packet[8]);

	Relativty::ServerDriver::Log("ARCH| SOCKET| RGB to int\n");
	int colID = colorToIdentity(rf, gf, bf);

	return positionPacket{ xf,yf,sf,colID, camId };
}

void Relativty::SocketServer::processInputPacket(Relativty::SocketServer::inputPacket packet) {
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
	freshState[i] = true;
}

Relativty::SocketServer::inputPacket Relativty::SocketServer::parseInputPacket(const char* packet) {
	//the proposed protocol for position packet follows :
	/*
			0	 1		2	3	4	5	6	7	8	9
		[Type=P][ID] + [B1][B2][B3][B4][B5][B6][B7][B8]
	*/

	int colID = int(packet[1]);

	return inputPacket{ int(packet[2]), float(packet[3]),float(packet[4]),float(packet[5]),
			float(packet[6]) ,float(packet[7]) ,float(packet[8]) ,float(packet[9]),
			float(packet[10]) };
}

char Relativty::SocketServer::rgbToChar(float R, float G, float B) {
	return colChar[colorToIdentity(R, G, B)];
}

int Relativty::SocketServer::colorToIdentity(float R, float G, float B)
{
	return (R >= G && R >= B) ? 0 : ((G >= R && G >= B) ? 1 : ((B >= R && B >= G) ? 2 : -1) );
}

int Relativty::SocketServer::charToIdentity(char ch) {
	return ch == 'R' ? 0 : (ch == 'G' ? 1 : (ch == 'B' ? 2 : -1)) ;
}

//Close the socket server, informing all connections.
void Relativty::SocketServer::close()
{
	closesocket(sock);
	this->cycle_receive_parse_packets_thread_worker.join();
	WSACleanup();
}

bool Relativty::SocketServer::isAvailable(int id) {
	bool isAvailable = freshState[id];
	return isAvailable;
}

bool Relativty::SocketServer::isAvailable(char id) {
	return isAvailable(charToIdentity(id));
}

Relativty::SocketServer::deviceState Relativty::SocketServer::getState(int id) {
	Relativty::ServerDriver::Log("ARCH| 2| got X " + std::to_string(coordinate[id][0]));
	Relativty::ServerDriver::Log("ARCH| 2| got Y " + std::to_string(coordinate[id][1]));
	Relativty::ServerDriver::Log("ARCH| 2| got Z " + std::to_string(coordinate[id][2]));
	float X = coordinate[id][0];
	float Y = coordinate[id][1];
	float Z = coordinate[id][2];
	float coord[3]{ X,Y,Z };
	float button[8]{ buttons[id][0],buttons[id][1],buttons[id][2],buttons[id][3],buttons[id][4],buttons[id][5],buttons[id][6], buttons[id][7] };
	freshState[id] = false;
	deviceState sta = deviceState{ coord, button };
	Relativty::ServerDriver::Log("ARCH| 2| got X2 " + std::to_string(sta.coordinate[0]));
	Relativty::ServerDriver::Log("ARCH| 2| got Y2 " + std::to_string(sta.coordinate[1]));
	Relativty::ServerDriver::Log("ARCH| 2| got Z2 " + std::to_string(sta.coordinate[2]));
	return sta;
}

Relativty::SocketServer::deviceState Relativty::SocketServer::getState(char id) {
	//Relativty::ServerDriver::Log("ARCH| SocketServer| get state from char to identity '" + std::to_string(id) + "' -> '" + std::to_string(charToIdentity(id)) + "'\n");
	return getState(charToIdentity(id));
}
#define _CRT_SECURE_NO_WARNINGS
//#define HAS_KABSCH
#include "Relativty_SocketServer.hpp"
#include "Relativty_ServerDriver.hpp"
#ifdef HAS_KABSCH
#include "Kabsch/KabschModule.hpp"
#endif // DEBUG
#include <WinSock2.h>
#include <string>
#include <io.h>
#include <limits.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <vector>

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
	#if CHAR_MIN < 0 
	Relativty::ServerDriver::Log("Plain char is signed\n");
	#endif
	Relativty::ServerDriver::Log("Loading Socket Server Settings...\n");
	static const char* const Relativty_hmd_section = "Relativty_general";
	this->devices = vr::VRSettings()->GetFloat(Relativty_hmd_section, "devices");
	Relativty::ServerDriver::Log("Done.\n");

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

	listen(this->sock, this->devices);

	if (!bIsStaticPosition) {
		Relativty::ServerDriver::Log("SocketServer: Waiting for " + std::to_string(this->devices) + " devices to connect...\n");
		for (int i = 0; i < this->devices; i++)
		{
			addressLen = sizeof(struct sockaddr_in);
			this->sock_receive[i] = accept(this->sock, (struct sockaddr*) & client, &addressLen);
			if (this->sock_receive[i] == INVALID_SOCKET)
				Relativty::ServerDriver::Log("SocketServer: accept failed with error code: " + WSAGetLastError());
			DWORD timeout = 1 * 10;
			setsockopt(this->sock_receive[i], SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
			sock_state[i] = true;
			Relativty::ServerDriver::Log("SocketServer: Connection #" + std::to_string(i) + " accepted");

		}
		/*
		*/
	}

	Relativty::ServerDriver::Log("SocketServer: Start packet parser thread 3.A:");
	this->cycle_receive_parse_packets_thread_worker = std::thread(&Relativty::SocketServer::cycle, this);
	Relativty::ServerDriver::Log("SocketServer: packet parser thread thread started");

	Relativty::ServerDriver::Log("SocketServer: packet parser thread 3.A \n");

	//this->cycle_receive_parse_packets_thread_worker.join();
	this->readyness = true;
}

void Relativty::SocketServer::precv(char incomingData[], const int incomingDataLength, bool r)
{
	if (incomingDataLength > 0) {
		const int partCount = incomingDataLength / 4;
		Relativty::SocketServer::packetPart pparts[428];
		//pparts.reserve(partCount);
		int pi = -1;
		for (int i = 0; i < incomingDataLength; i += 4)
		{
			pi++;
			//grab 4 bytes 
			byte a = incomingData[i];
			byte b = incomingData[i + 1];
			byte c = incomingData[i + 2];
			byte d = incomingData[i + 3];
			std::string str = std::to_string(a) + " " + std::to_string(b) + " " + std::to_string(c) + " " + std::to_string(d);
			Relativty::SocketServer::packetPart ppart;
			if (r) {
				ppart = Relativty::SocketServer::packetPart({ c, d, a, b });
			}
			else {
				ppart = Relativty::SocketServer::packetPart({ a, b, c, d });
			}
			Relativty::ServerDriver::Log("ARCH| part composed '" + str + "' -> '" + ppart.getChar() + "' or '" + std::to_string(ppart.getInt()) + "'\n");
			pparts[pi] = ppart;
			Relativty::ServerDriver::Log("ARCH| part composed after\n");

		}
		//compose packet :
		int packetsFound = 0;
		int iFrom = -1;
		int iTo = -1;
		int i = 0;
		bool isFindingStart = true;
		while (i < partCount) {
			if (isFindingStart)
				if (pparts[i].isChar('{')) {
					Relativty::ServerDriver::Log("ARCH| open bracket start'\n");
					iFrom = i;
					isFindingStart = false;
					i++;
					continue;
				}
			if (!isFindingStart) {
				if (pparts[i].isChar('{')) {
					Relativty::ServerDriver::Log("ARCH| bad packet'\n");
					iFrom = -1;
					isFindingStart = true;
					i++;
					continue;
				}
				else if (pparts[i].isChar('}')) {
					Relativty::ServerDriver::Log("ARCH| close bracket end'\n");
					iTo = i;
					packetsFound++;
					Relativty::SocketServer::sockPacket packet = sockPacket();
					packet.compose(pparts, iFrom, iTo);
					Relativty::ServerDriver::Log("ARCH| Parse and process packet #" + std::to_string(packetsFound) + "'\n");
					parsePacket((Relativty::SocketServer::sockPacket)packet);
					isFindingStart = true;
					i++;
					continue;
				}
			}
			i++;
		}
		if (!isFindingStart)
			Relativty::ServerDriver::Log("ARCH| incomplete packet. Discarding'\n");
	}
}

void Relativty::SocketServer::precv(int i) {
	const int receiveBufferLen = 1536;
	char receiveBuffer[receiveBufferLen];

	if (sock_state[i] == false) {
		Relativty::ServerDriver::Log("SocketServer: Socket #" + std::to_string(i) + " is closed. attempting to re open...\n");
		//attempt to recconect socket, else retur early.
		int addressLen = sizeof(struct sockaddr_in);
		struct sockaddr_in client;
		try
		{
			this->sock_receive[i] = accept(this->sock, (struct sockaddr*) & client, &addressLen);
			if (this->sock_receive[i] == INVALID_SOCKET) {
				Relativty::ServerDriver::Log("SocketServer: accept failed with error code: " + std::to_string(WSAGetLastError()) + "\n");
				return;
			}
			DWORD timeout = 1 * 10;
			setsockopt(this->sock_receive[i], SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
			sock_state[i] = true;

			Relativty::ServerDriver::Log("SocketServer: Connection on Socket #" + std::to_string(i) + " accepted\n");
		}
		catch (const std::exception&)
		{
			sock_state[i] = false;
			Relativty::ServerDriver::Log("SocketServer: failed to recconect on Socket #" + std::to_string(i) + "\n");
		}
	}
	
	//Relativty::ServerDriver::Log("ARCH| SOCKET| check #" + std::to_string(i) + "~ \n");
	if (sock_state[i] == false) {
		Relativty::ServerDriver::Log("SocketServer: Socket #" + std::to_string(i) + " not setablished : " + std::to_string(WSAGetLastError()));
		return;
	}

	const int resultReceiveLen = recv(this->sock_receive[i], receiveBuffer, receiveBufferLen, NULL);

	if (resultReceiveLen == -1) {
		fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
		Relativty::ServerDriver::Log("ARCH| SOCKET| recv error [" + std::string(strerror(errno)) + "].");
		return;
	}
	if (resultReceiveLen == 0) {
		Relativty::ServerDriver::Log("SocketServer: Received nothing in last recv call. assuming socket is closed");
		sock_state[i] = false;
		return;
	}
	precv(receiveBuffer, resultReceiveLen);
}

void Relativty::SocketServer::cycle()
{
	for (int i = 0; i < 3;i++) {
		coordinate[i][0] = 0;
		coordinate[i][1] = 0;
		coordinate[i][2] = 0;
	}
	for (int i = 0; i < 3;i++) {
		rotation[i][0] = 0;
		rotation[i][1] = 0;
		rotation[i][2] = 0;
		rotation[i][3] = 0;
	}
	for (int i = 0; i < 3;i++) {
		buttons[i][0] = 0;
		buttons[i][1] = 0;
		buttons[i][2] = 0;
		buttons[i][3] = 0;
		buttons[i][4] = 0;
		buttons[i][5] = 0;
		buttons[i][6] = 0;
		buttons[i][7] = 0;
	}

	Relativty::ServerDriver::Log("ARCH| SOCKET| CYCLE START!!\n");
	while (this->run_thread) {
		//Relativty::ServerDriver::Log("ARCH| SOCKET| CYCLE\n");
		for (int i = 0; i < this->devices; i++)
		{
			precv(i);
		}
	}
	Relativty::ServerDriver::Log("ARCH| HMD| SOCKET| thread ended!!\n");
}

void Relativty::SocketServer::parsePacket(Relativty::SocketServer::sockPacket packet) {
	//the proposed protocol for any packet follows :
	/*
		[Type=P][ID] + [X][Y][S][R][G][B]
		OR
		[Type=I][ID] + [B1][B2][B3][B4][B5][B6][B7][B8]

	*/
	char packetType = packet.getChar(1);
	std::string strr = std::to_string(packetType);
	Relativty::ServerDriver::Log("ARCH| SOCKET| packet type value '" + strr + "'\n");
	switch (packetType)
	{
	case 'P':
		Relativty::ServerDriver::Log("ARCH| SOCKET| packet type 'Position'\n");
#ifdef HAS_KABSCH
		if (_USE_V4_PROTOCOL_) {
			parsePositionPacketFrame(packet);
		}
		else {
#endif
			positionPacket pposition = parsePositionPacket(packet);
			processPositionPacket(pposition);
#ifdef HAS_KABSCH
		}
#endif
		break;
	case 'I':
		Relativty::ServerDriver::Log("ARCH| SOCKET| packet type 'Input'\n");
		inputPacket pinput = parseInputPacket(packet);
		processInputPacket(pinput);
		break;
	case 'R':
		Relativty::ServerDriver::Log("ARCH| SOCKET| packet type 'Rotation'\n");
		rotationPacket protation = parseRotationPacket(packet);
		processRotationPacket(protation);
		break;
	default:
		Relativty::ServerDriver::Log("ARCH| received invalid packet type '" + std::to_string(packetType) + "'. expected 'P' or 'I' or 'R' .\n");
		break;
	}
}

#ifdef HAS_KABSCH
//v4 protocols :
void Relativty::SocketServer::parsePositionPacketFrame(Relativty::SocketServer::sockPacket packet) {
	// This packet is dynamic in size, we have no prederminate size and must build expectation from the packet :
	//	[{][P][ID][COUNT]
	//	 0   1   2   3   4   5   6   7   8   9   10  11
	//  [X1][Y1][Z1][X2][Y2][Z2][X3][Y3][Z3][X4][Y4][Z4]

	int camId = int(packet.getInt(2));
	Relativty::ServerDriver::Log("ARCH| SOCKET| begin parse position packet v4\n");
	int packetType = int(packet.getInt(1));

	int pointCount = int(packet.getInt(3));
	Relativty::ServerDriver::Log("ARCH| SOCKET| number of blobs:" + std::to_string(pointCount)+ "\n");
	//every 3 ints is a kabsch Point :

	std::vector<Eigen::Vector3f> Points;
	Points.resize(pointCount);
	for (int i = 0; i <= pointCount; i++)
	{
		float X = float(packet.getInt(4 + (i * 3) + 0));
		float Y = float(packet.getInt(4 + (i * 3) + 1));
		float Z = float(packet.getInt(4 + (i * 3) + 2));
		
		Points.push_back(Eigen::Vector3f(X, Y, Z));
	}
	CameraPointsPerFrame[camId] = Points;
	//updated Camera points. on demand kabsch will recalculate instead of preprocessing.
	//that way only the moment a pose is demanded will heavier logic run?
}
//
#endif

void Relativty::SocketServer::processPositionPacket(positionPacket packet) {
	//process the packet into coordinates.
	Relativty::ServerDriver::Log("ARCH| SOCKET| Process position packet start\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| packet came from camera #" + std::to_string(packet.CamID) + "\n");
	if (packet.CamID == 1) {//the front side camera
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates on " + std::to_string(packet.colID) + "\n");

		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates X " + std::to_string(packet.X * -1) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates Y " + std::to_string(coordinate[packet.colID][1]) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates Z " + std::to_string(packet.Y * -1) + "\n");
		coordinate[packet.colID][0] = (packet.X * -1);// -(2592.0f / 2.0f); //fwd back
		coordinate[packet.colID][1] = float(coordinate[packet.colID][1]); //lr?
		coordinate[packet.colID][2] = float(coordinate[packet.colID][2]);//packet.Y*-1; //height

		//Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set X " + std::to_string(coordinate[packet.colID][0]) + "\n");
		//Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set Y " + std::to_string(coordinate[packet.colID][1]) + "\n");
		//Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set Z " + std::to_string(coordinate[packet.colID][2]) + "\n");
	}
	if (packet.CamID == 2) {//the right side camera
		float sadjusted = (-packet.S / 9.6f)-3;
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates on " + std::to_string(packet.colID) + "\n");

		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates X " + std::to_string(sadjusted) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates Y " + std::to_string(packet.X) + "\n");
		Relativty::ServerDriver::Log("ARCH| SOCKET| set coordinates Z " + std::to_string(packet.Y) + "\n");

		//coordinate[packet.colID][0] = float(coordinate[packet.colID][0]);
		coordinate[packet.colID][0] = sadjusted;
		coordinate[packet.colID][1] = packet.X;// -(2592.0f / 2.0f);
		coordinate[packet.colID][2] = packet.Y;// -(1944.0f / 2.0f);//float(coordinate[packet.colID][2]);//packet.Y;

		//Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set X " + std::to_string(coordinate[packet.colID][0]) + "\n");
		//Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set Y " + std::to_string(coordinate[packet.colID][1]) + "\n");
		//Relativty::ServerDriver::Log("ARCH| SOCKET| coordinates set Z " + std::to_string(coordinate[packet.colID][2]) + "\n");
	}
	hasNewCoordinatesState[packet.colID] = true;
	Relativty::ServerDriver::Log("ARCH| SOCKET| Process position packet end\n");
}

Relativty::SocketServer::positionPacket Relativty::SocketServer::parsePositionPacket(Relativty::SocketServer::sockPacket packet) {
	//the proposed protocol for position packet follows :
	/*
			0	 1		2  3  4  5  6  7
		[Type=P][ID] + [X][Y][S][R][G][B]
	*/

	Relativty::ServerDriver::Log("ARCH| SOCKET| begin parse position packet\n");
	
	int packetType = int(packet.getInt(1));
	Relativty::ServerDriver::Log("ARCH| SOCKET| parse position packet ID\n");
	int camId = int(packet.getInt(2));
	Relativty::ServerDriver::Log("ARCH| SOCKET| got ID " + std::to_string(camId) + "\n");


	Relativty::ServerDriver::Log("ARCH| SOCKET| parse position packet XY\n");
	float xf = float(packet.getInt(3));
	float yf = float(packet.getInt(4));
	Relativty::ServerDriver::Log("ARCH| SOCKET| got (" + std::to_string(xf) + " , " + std::to_string(yf) + ")");

	Relativty::ServerDriver::Log("ARCH| SOCKET| parse position packet S\n");
	float sf = float(packet.getInt(5));

	Relativty::ServerDriver::Log("ARCH| SOCKET| parse position packet RGB\n");
	float rf = float(packet.getInt(6));
	float gf = float(packet.getInt(7));
	float bf = float(packet.getInt(8));
	Relativty::ServerDriver::Log("ARCH| SOCKET| go RGB (" + std::to_string(rf) + " , "  + std::to_string(gf) + " , " + std::to_string(bf) + ")\n");

	Relativty::ServerDriver::Log("ARCH| SOCKET| RGB to int\n");
	int colID = colorToIdentity(rf, gf, bf);

	return positionPacket{ xf,yf,sf,colID, camId };
}

void Relativty::SocketServer::processInputPacket(Relativty::SocketServer::inputPacket packet) {
	//process the packet into coordinates.
	Relativty::ServerDriver::Log("ARCH| SOCKET| set inputs on " + std::to_string(packet.colID) + "\n");
	const int i = packet.colID;

	Relativty::ServerDriver::Log("ARCH| SOCKET| FETCH  INPUT A :" + std::to_string(packet.B1) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| FETCH  INPUT B :" + std::to_string(packet.B2) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| FETCH  INPUT C :" + std::to_string(packet.B3) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| FETCH  INPUT D :" + std::to_string(packet.B4) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| FETCH  INPUT E :" + std::to_string(packet.B5) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| FETCH  INPUT F :" + std::to_string(packet.B6) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| FETCH  INPUT G :" + std::to_string(packet.B7) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| FETCH  INPUT H :" + std::to_string(packet.B8) + "\n");

	buttons[packet.colID][0] = packet.B1;
	buttons[packet.colID][1] = packet.B2;
	buttons[packet.colID][2] = packet.B3;
	buttons[packet.colID][3] = packet.B4;
	buttons[packet.colID][4] = packet.B5;
	buttons[packet.colID][5] = packet.B6;
	buttons[packet.colID][6] = packet.B7;
	buttons[packet.colID][7] = packet.B8;
	

	Relativty::ServerDriver::Log("ARCH| SOCKET| SET INPUT A :" + std::to_string(buttons[i][0]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SET INPUT B :" + std::to_string(buttons[i][1]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SET INPUT C :" + std::to_string(buttons[i][2]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SET INPUT D :" + std::to_string(buttons[i][3]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SET INPUT E :" + std::to_string(buttons[i][4]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SET INPUT F :" + std::to_string(buttons[i][5]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SET INPUT G :" + std::to_string(buttons[i][6]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SET INPUT H :" + std::to_string(buttons[i][7]) + "\n");

	Relativty::ServerDriver::Log("ARCH| SOCKET| Process inputs packet end\n");
	hasNewInputState[i] = true;
}

Relativty::SocketServer::inputPacket Relativty::SocketServer::parseInputPacket(sockPacket packet) {
	//the proposed protocol for position packet follows :
	/*
			0	 1		2	3	4	5	6	7	8	9
		[Type=I][ID] + [B1][B2][B3][B4][B5][B6][B7][B8]
	*/
	int colID = packet.getInt(2) - '0';
	Relativty::ServerDriver::Log("ARCH| SOCKET| input for device # :" + std::to_string(packet.getInt(2) - '0') + "\n");
	int A = packet.getInt(3) - '0';
	int B = packet.getInt(4) - '0';
	int C = packet.getInt(5) - '0';
	int D = packet.getInt(6) - '0';
	int E = packet.getInt(7) - '0';
	int F = packet.getInt(8) - '0';
	int G = packet.getInt(9) - '0';
	int H = packet.getInt(10) - '0';

	Relativty::SocketServer::inputPacket ppacket = Relativty::SocketServer::inputPacket{ colID,A,B,C,D,E,1,G,A };

	//Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT A :" + std::to_string(packet.) +"\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT A :" + std::to_string(ppacket.B1) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT B :" + std::to_string(ppacket.B2) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT C :" + std::to_string(ppacket.B3) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT D :" + std::to_string(ppacket.B4) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT E :" + std::to_string(ppacket.B5) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT F :" + std::to_string(ppacket.B6) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT G :" + std::to_string(ppacket.B7) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| INPUT H :" + std::to_string(ppacket.B8) + "\n");

	return ppacket;
}

void Relativty::SocketServer::processRotationPacket(std::string packet) {
	nlohmann::json parser = nlohmann::json::parse(packet.c_str());
	//first verify packet has all fields (WXYZC)
	if (!parser.contains("X") || !parser.contains("Y") || !parser.contains("Z") || !parser.contains("W") || !parser.contains("C")) {
		Relativty::ServerDriver::Log("ARCH| JSON| process rotation snippet failed because :\n");
		Relativty::ServerDriver::Log("ARCH| JSON|     HAS X key :" + std::to_string(parser.contains("X")) + "\n");
		Relativty::ServerDriver::Log("ARCH| JSON|     HAS Y key :" + std::to_string(parser.contains("Y")) + "\n");
		Relativty::ServerDriver::Log("ARCH| JSON|     HAS Z key :" + std::to_string(parser.contains("Z")) + "\n");
		Relativty::ServerDriver::Log("ARCH| JSON|     HAS W key :" + std::to_string(parser.contains("W")) + "\n");
		Relativty::ServerDriver::Log("ARCH| JSON|     HAS C key :" + std::to_string(parser.contains("C")) + "\n");
		return;
	}
	//Relativty::ServerDriver::Log("ARCH| JSON| process rotation snippet succeeded for :" + std::to_string(parser["C"].get<int>()) + "\n");
	//Relativty::ServerDriver::Log("ARCH| JSON|     HAS X key :" + std::to_string(parser.contains("X")) + "\n");
	//Relativty::ServerDriver::Log("ARCH| JSON|     HAS Y key :" + std::to_string(parser.contains("Y")) + "\n");
	//Relativty::ServerDriver::Log("ARCH| JSON|     HAS Z key :" + std::to_string(parser.contains("Z")) + "\n");
	//Relativty::ServerDriver::Log("ARCH| JSON|     HAS W key :" + std::to_string(parser.contains("W")) + "\n");
	//Relativty::ServerDriver::Log("ARCH| JSON|     HAS C key :" + std::to_string(parser.contains("C")) + "\n");
	int i = parser["C"].get<int>();
	rotation[i][0] = parser["W"].get<float>();
	rotation[i][1] = parser["X"].get<float>();
	rotation[i][2] = parser["Y"].get<float>();
	rotation[i][3] = parser["Z"].get<float>();
	//Relativty::ServerDriver::Log("ARCH| JSON| SSET ROTATION W :" + std::to_string(rotation[i][0]) + "\n");
	//Relativty::ServerDriver::Log("ARCH| JSON| SSET ROTATION X :" + std::to_string(rotation[i][1]) + "\n");
	//Relativty::ServerDriver::Log("ARCH| JSON| SSET ROTATION Y :" + std::to_string(rotation[i][2]) + "\n");
	//Relativty::ServerDriver::Log("ARCH| JSON| SSET ROTATION Z :" + std::to_string(rotation[i][3]) + "\n");
	hasNewRotationState[i] = true;
}

void Relativty::SocketServer::processRotationPacket(Relativty::SocketServer::rotationPacket packet) {
	//process the packet into coordinates.
	Relativty::ServerDriver::Log("ARCH| SOCKET| set ROTATION on " + std::to_string(packet.colID) + "\n");
	const int i = packet.colID;
	rotation[i][0] = (float(packet.W) / 1000.0f) -1;
	rotation[i][1] = (float(packet.X) / 1000.0f) -1;
	rotation[i][2] = (float(packet.Y) / 1000.0f) -1;
	rotation[i][3] = (float(packet.Z) / 1000.0f) -1;

	Relativty::ServerDriver::Log("ARCH| SOCKET| SSET ROTATION W :" + std::to_string(rotation[i][0]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SSET ROTATION X :" + std::to_string(rotation[i][1]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SSET ROTATION Y :" + std::to_string(rotation[i][2]) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| SSET ROTATION Z :" + std::to_string(rotation[i][3]) + "\n");
	hasNewRotationState[i] = true;
	Relativty::ServerDriver::Log("ARCH| SOCKET| Process ROTATION packet end\n");
}

Relativty::SocketServer::rotationPacket Relativty::SocketServer::parseRotationPacket(Relativty::SocketServer::sockPacket packet) {
	//the proposed protocol for rotation packet follows :
	/*
			1	 2		3  4  5  6
		[Type=R][ID] + [W][X][Y][Z]
	*/
	int colID = charToIdentity(packet.getInt(2));
	Relativty::ServerDriver::Log("ARCH| SOCKET| ROTATION for device # :" + std::to_string(identityToChar(colID)) + "(" + std::to_string(colID) + ")\n");
	int W = (packet.getInt(3));
	int X = (packet.getInt(4));
	int Y = (packet.getInt(5));
	int Z = (packet.getInt(6));
	Relativty::SocketServer::rotationPacket rpacket = Relativty::SocketServer::rotationPacket{ colID, W, X ,Y, Z };

	Relativty::ServerDriver::Log("ARCH| SOCKET| ROTATION W :" + std::to_string(rpacket.W) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| ROTATION X :" + std::to_string(rpacket.X) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| ROTATION Y :" + std::to_string(rpacket.Y) + "\n");
	Relativty::ServerDriver::Log("ARCH| SOCKET| ROTATION Z :" + std::to_string(rpacket.Z) + "\n");

	return rpacket;
}

char Relativty::SocketServer::rgbToChar(float R, float G, float B) {
	return colChar[colorToIdentity(R, G, B)];
}

int Relativty::SocketServer::colorToIdentity(float R, float G, float B)
{
	return (R >= G && R >= B) ? 0 : ((G >= R && G >= B) ? 1 : ((B >= R && B >= G) ? 2 : -1) );
}

int Relativty::SocketServer::charToIdentity(char ch) {
	return (ch == 'R' || ch == '0') ? 0 : ((ch == 'G' || ch == '1') ? 1 : ((ch == 'B' || ch == '2') ? 2 : -1)) ;
}

char Relativty::SocketServer::identityToChar(int i) {
	return (i == 0 || i == '0') ? 'R' : ((i == 1 || i == '1') ? 'G' : ((i == 2 || i == '2') ? 'B' : 'N'));
}

//Close the socket server, informing all connections.
void Relativty::SocketServer::close()
{
	closesocket(sock);
	this->cycle_receive_parse_packets_thread_worker.join();
	WSACleanup();
}

bool Relativty::SocketServer::isNewRotation(int id) {
	bool isAvailable = hasNewRotationState[id];
	return isAvailable;
}

bool Relativty::SocketServer::isNewCoordinates(int id) {
	bool isAvailable = hasNewCoordinatesState[id];
	return isAvailable;
}

bool Relativty::SocketServer::isNewInputs(int id) {
	bool isAvailable = hasNewInputState[id];
	return isAvailable;
}

bool Relativty::SocketServer::isNewRotation(char id) {
	return isNewRotation(charToIdentity(id));
}
bool Relativty::SocketServer::isNewCoordinates(char id) {
	return isNewCoordinates(charToIdentity(id));
}
bool Relativty::SocketServer::isNewInputs(char id) {
	return isNewInputs(charToIdentity(id));
}

Relativty::SocketServer::coordinateState Relativty::SocketServer::getCoordinateState(char id) {
	return getCoordinateState(charToIdentity(id));
}

Relativty::SocketServer::rotationState Relativty::SocketServer::getRotationState(char id) {
	return getRotationState(charToIdentity(id));
}

Relativty::SocketServer::inputState Relativty::SocketServer::getInputState(char id) {
	return getInputState(charToIdentity(id));
}

Relativty::SocketServer::inputState Relativty::SocketServer::getInputState(int id) {
	float A = buttons[id][0] + 0;
	float B = buttons[id][1] + 0;
	float C = buttons[id][2] + 0;
	float D = buttons[id][3] + 0;
	float E = buttons[id][4] + 0;
	float F = buttons[id][5] + 0;
	float G = buttons[id][6] + 0;
	float H = buttons[id][7] + 0;
	float button[8]{ A,B,C,D,E,F,1.0f,H };
	inputState sta = inputState{ button };
	Relativty::ServerDriver::Log("ARCH| get state| got A :" + std::to_string(sta.button[0]) + "\n");
	Relativty::ServerDriver::Log("ARCH| get state| got B :" + std::to_string(sta.button[1]) + "\n");
	Relativty::ServerDriver::Log("ARCH| get state| got C :" + std::to_string(sta.button[2]) + "\n");
	Relativty::ServerDriver::Log("ARCH| get state| got D :" + std::to_string(sta.button[3]) + "\n");
	Relativty::ServerDriver::Log("ARCH| get state| got E :" + std::to_string(sta.button[4]) + "\n");
	Relativty::ServerDriver::Log("ARCH| get state| got F :" + std::to_string(sta.button[5]) + "\n");
	Relativty::ServerDriver::Log("ARCH| get state| got G :" + std::to_string(sta.button[6]) + "\n");
	Relativty::ServerDriver::Log("ARCH| get state| got H :" + std::to_string(sta.button[7]) + "\n");
	hasNewInputState[id] = false;
	return sta;
}

Relativty::SocketServer::rotationState Relativty::SocketServer::getRotationState(int id) {
	Relativty::ServerDriver::Log("ARCH| get state|1 got r W " + std::to_string(rotation[id][0]));
	Relativty::ServerDriver::Log("ARCH| get state|1 got r X " + std::to_string(rotation[id][1]));
	Relativty::ServerDriver::Log("ARCH| get state|1 got r Y " + std::to_string(rotation[id][2]));
	Relativty::ServerDriver::Log("ARCH| get state|1 got r Z " + std::to_string(rotation[id][3]));

	float rW = rotation[id][0];
	float rX = rotation[id][1];
	float rY = rotation[id][2];
	float rZ = rotation[id][3];

	float rotate[4]{ rW, rX, rY, rZ };
	hasNewRotationState[id] = false;
	rotationState sta = rotationState{ rotate };
	Relativty::ServerDriver::Log("ARCH| get state| got r W " + std::to_string(sta.rotation[0]));
	Relativty::ServerDriver::Log("ARCH| get state| got r X " + std::to_string(sta.rotation[1]));
	Relativty::ServerDriver::Log("ARCH| get state| got r Y " + std::to_string(sta.rotation[2]));
	Relativty::ServerDriver::Log("ARCH| get state| got r Z " + std::to_string(sta.rotation[3]));
	hasNewRotationState[id] = false;
	return sta;
}

Relativty::SocketServer::coordinateState Relativty::SocketServer::getCoordinateState(int id) {
	Relativty::ServerDriver::Log("ARCH| 2| got X " + std::to_string(coordinate[id][0]));
	Relativty::ServerDriver::Log("ARCH| 2| got Y " + std::to_string(coordinate[id][1]));
	Relativty::ServerDriver::Log("ARCH| 2| got Z " + std::to_string(coordinate[id][2]));
	float X = coordinate[id][0];
	float Y = coordinate[id][1];
	float Z = coordinate[id][2];
	float coord[3]{ X,Y,Z };
	coordinateState sta = coordinateState{ coord };
	Relativty::ServerDriver::Log("ARCH| get state| got X2 " + std::to_string(sta.coordinate[0]));
	Relativty::ServerDriver::Log("ARCH| get state| got Y2 " + std::to_string(sta.coordinate[1]));
	Relativty::ServerDriver::Log("ARCH| get state| got Z2 " + std::to_string(sta.coordinate[2]));
	hasNewCoordinatesState[id] = false;
	return sta;
}
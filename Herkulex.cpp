/*
 * Herkulex.cpp
 *
 *  Created on: 2016. 6. 16.
 *      Author: em
 */

#include <stdio.h>

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>

#include <device/Property.h>
#include <device/ApiTypes.h>
#include "Herkulex.h"

#define SUCCESS 1
#define FAIL 0

#define DEBUG

int Herkulex::uart_init(const char* portName, int baudrate) {
#ifdef DEBUG
	std::cout << portName << " " << fd << std::endl;
#endif

	fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
//	fd = open(portName, O_RDWR | O_SYNC);

	tcgetattr(fd, &tio);
	tcflush(fd, TCIOFLUSH);

	switch (baudrate) {
	case 9600:
		cfsetispeed(&tio, B9600);
		cfsetospeed(&tio, B9600);
		break;
	case 19200:
		cfsetispeed(&tio, B19200);
		cfsetospeed(&tio, B19200);
		break;
	case 38400:
		cfsetispeed(&tio, B38400);
		cfsetospeed(&tio, B38400);
		break;
	case 115200:
		cfsetispeed(&tio, B115200);
		cfsetospeed(&tio, B115200);
		break;
	default:
		cfsetispeed(&tio, B115200);
		cfsetospeed(&tio, B115200);
		break;
	}

//	tio.c_cflag |= CLOCAL;
//	tio.c_cflag |= CREAD;
//
//	tio.c_cflag &= ~CSIZE; // clear frame size info
//	tio.c_cflag |= CS8;    // 8 bit frames
//	tio.c_cflag &= ~PARENB;    // no parity
//	tio.c_cflag &= ~CSTOPB;    // one stop bit

	tio.c_cflag |= CS8;
	tio.c_cflag |= CREAD | HUPCL | CLOCAL;

	tio.c_iflag = IGNBRK | IGNPAR;
	tio.c_oflag = ONLRET | ONOCR;

	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;

	tio.c_line = 0;

	tcsetattr(fd, TCSANOW, &tio);
	tcflush(fd, TCIOFLUSH);

	return SUCCESS;
}

void Herkulex::uart_close() {
#ifdef DEBUG
	std::cout << "Herkulex::uart_close()" << std::endl;
#endif
	close(fd);
}
int Herkulex::uartTx(const unsigned char* data, int size) {
#ifdef DEBUG
	std::cout << "Herkulex::uartTX : " << ":" << size << std::endl;
#endif
	ssize_t txSize = 0;
	txSize = write(fd, data, (size_t) size);

	usleep(2000);

	return (int) txSize;
}

int Herkulex::uartRx(unsigned char* data, int size) {
#ifdef DEBUG
	std::cout << "Herkulex::uartRX() : " << ":" << data << ":" << size
			<< std::endl;
#endif
	ssize_t rxSize = 0;
	rxSize = read(fd, (void*) data, (size_t) size);

	if (rxSize >= 0)
		return rxSize;
	else
		return FAIL;

}

int Herkulex::setBaudrate(int baudrate) {
#ifdef DEBUG
	std::cout << "Herkulex::set Baudrate()" << std::endl;
#endif
	return SUCCESS;
}

/****************** Herkulex Function ****************//*
 Header PacketSize servoID CMD CheckSum1 CheckSum2 Data[n]
 2		   1	     1	  1	  	 1			 1			n
 */

void Herkulex::TorquePolicy(uint8_t servoID, uint8_t state) {
	const uint8_t packetSize = 10;
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t packet[10] = { 0xFF, 0xFF, packetSize, servoID, RAM_WRITE,
			checkSum1, checkSum2, TORQUE_RAM_ADDR, 0x01, state };

	checkSum1 = (packet[2] ^ packet[3] ^ packet[4] ^ packet[7] ^ packet[8]
			^ packet[9]) & 0xFE;
	checkSum2 = (~checkSum1) & 0xFE;

	packet[5] = checkSum1;
	packet[6] = checkSum2;

		uartTx(packet, packetSize);
}

void Herkulex::SetLED(uint8_t servoID, uint8_t color) {
	const uint8_t packetSize = 10;
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t packet[10] = { 0xFF, 0xFF, packetSize, servoID, RAM_WRITE,
			checkSum1, checkSum2, LED_RAM_ADDR, 0x01, color };

	checkSum1 = (packet[2] ^ packet[3] ^ packet[4] ^ packet[7] ^ packet[8]
			^ packet[9]) & 0xFE;
	checkSum2 = (~checkSum1) & 0xFE;

	packet[5] = checkSum1;
	packet[6] = checkSum2;

	uartTx(packet, (int) packetSize);
}

void Herkulex::ClearError(uint8_t servoID) {
	const uint8_t packetSize = 0x0B;
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t packet[11] = { 0xFF, 0xFF, packetSize, servoID, RAM_WRITE,
			checkSum1, checkSum2, STATUS_ERROR_ADDR, 0x02, 0x00, 0x00 };

	checkSum1 = (packet[2] ^ packet[3] ^ packet[4] ^ packet[7] ^ packet[8]
			^ packet[9] ^ packet[10]) & 0xFE;
	checkSum2 = (~checkSum1) & 0xFE;

	packet[5] = checkSum1;
	packet[6] = checkSum2;

	uartTx(packet, (int) packetSize);

}

void Herkulex::GetError(uint8_t servoID) {
	uint8_t recv[30] = { 0, };
	const uint8_t packetSize = 12;
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t packet[12] = { 0xFF, 0xFF, packetSize, servoID, 0x44, checkSum1,
			checkSum2, 35, 0x01, 0x00, 0x42 };

	checkSum1 = (packetSize ^ servoID ^ RAM_WRITE ^ packet[7] ^ packet[8]
			^ packet[9] ^ packet[10]) & 0xFE;
	checkSum2 = (~checkSum1) & 0xFE;

	packet[5] = checkSum1;
	packet[6] = checkSum2;

	uartTx(packet, (int) packetSize);
	uartRx(recv, 30);
}

void Herkulex::Reboot(uint8_t servoID) {
	const uint8_t packetSize = 7;
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t packet[7] = { 0xFF, 0xFF, packetSize, servoID, REBOOT, checkSum1,
			checkSum2 };

	checkSum1 = (packet[2] ^ packet[3] ^ packet[4]) & 0xFE;
	checkSum2 = (~checkSum1) & 0xFE;

	packet[5] = checkSum1;
	packet[6] = checkSum2;

	uartTx(packet, (int) packetSize);
}

// playTime : [msec] , motor 16ea
void Herkulex::Moving(int* goal, int* playTime, uint8_t* color) {
	const uint8_t packetSize = 87;	// 7 + 5*16
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t upper = 0;
	uint8_t lower = 0;
	uint8_t timeByte = 0;
	uint8_t led = 0;

	uint8_t packet[87] = { 0xFF, 0xFF, packetSize, BROADCAST_ID, I_JOG,
			checkSum1, checkSum2, lower, upper, led, /* id */0, timeByte };

	for (int i = 0; i < 16; ++i) {
		int idx = i * 5;

		littleEndian(&packet[idx + 7], &packet[idx + 8], goal[i]);

		packet[idx + 9] = led | (color[i] << 2);
		packet[idx + 10] = i;

		if (playTime[i] >= 2856)
			packet[idx + 11] = 0xFF;
		else if (playTime[i] <= 0)
			packet[idx + 11] = 0x00;
		else
			packet[idx + 11] = (uint8_t) ((float) playTime[i] / 11.2f);
	}

	checkSum1 = CheckSum1(packet, packetSize);
	checkSum2 = CheckSum2(checkSum1);

	packet[5] = checkSum1;
	packet[6] = checkSum2;

	uartTx(packet, (int) packetSize);
}

// playTime : [msec]
void Herkulex::MovingSync(int* goal, uint8_t* color, int playTime) {
	// servo 16 ea
	const uint8_t packetSize = 72;	// 8 + 4 * 16
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t upper = 0;
	uint8_t lower = 0;
	uint8_t timeByte = 0;
	uint8_t led = 0;

	uint8_t packet[72] = { 0xFF, 0xFF, packetSize, 0xFE, S_JOG, checkSum1,
			checkSum2, timeByte, lower, upper, led, /*id*/};

	if (playTime >= 2856)
		timeByte = 0xFF;
	else if (playTime <= 0)
		timeByte = 0x00;
	else
		timeByte = (uint8_t) ((float) playTime / 11.2f);

	packet[7] = timeByte;

	for (int i = 0; i < 16; ++i) {
		// LSB MSB SET ID
		int idx = i * 4;

		littleEndian(&packet[idx + 8], &packet[idx + 9], goal[i]);
		packet[idx + 10] = led | (color[i] << 2);
		packet[idx + 11] = (uint8_t) i;
	}

	checkSum1 = CheckSum1(packet, packetSize);
	checkSum2 = CheckSum2(checkSum1);

	packet[5] = checkSum1;
	packet[6] = checkSum2;

	uartTx(packet, (int) packetSize);
}

// playTime : [msec]
void Herkulex::MovingOne(int* goal, int playTime, uint8_t* color) {
	// servo 16 ea
	const uint8_t packetSize = 72;	// 8 + 4 * 16
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t upper = 0;
	uint8_t lower = 0;
	uint8_t timeByte = 0;
	uint8_t led = 0;

	uint8_t packet[packetSize] = { 0xFF, 0xFF, packetSize, 0xFE, S_JOG,
			checkSum1, checkSum2, timeByte, lower, upper, led, /*id*/};

	if (playTime >= 2856)
		timeByte = 0xFF;
	else if (playTime <= 0)
		timeByte = 0x00;
	else
		timeByte = (uint8_t) ((float) playTime / 11.2f);

	packet[7] = timeByte;

	for (int i = 0; i < 16; ++i) {
		// LSB MSB SET ID
		int idx = i * 4;

		littleEndian(&packet[idx + 8], &packet[idx + 9], goal[i]);
		packet[idx + 10] = led | (color[i] << 2);
		packet[idx + 11] = (uint8_t) i;
	}

	checkSum1 = CheckSum1(packet, packetSize);
	checkSum2 = CheckSum2(checkSum1);

	packet[5] = checkSum1;
	packet[6] = checkSum2;

	uartTx(packet, (int) packetSize);
}

uint8_t Herkulex::CheckSum1(uint8_t* packet, int size) {
	int i;
	uint8_t CheckSum = 0;

	for (i = 2; i < size; ++i) {
		if (i == 5 || i == 6)
			continue;

		CheckSum ^= packet[i];
	}

	return (CheckSum & 0xFE);
}

uint8_t Herkulex::CheckSum2(uint8_t checkSum1) {

	return (~checkSum1) & 0xFE;
}

void Herkulex::littleEndian(uint8_t* lsb, uint8_t* msb, int data) {
	*msb = (0xFF00 & data) >> 8;
	*lsb = (0x00FF & data);
}

void Herkulex::SendCommand(uint8_t servoID, uint8_t cmd, uint8_t* optionalData,
		int dataSize) {
	uint8_t checkSum1 = 0;
	uint8_t checkSum2 = 0;
	uint8_t packetSize = 7 + dataSize;
	uint8_t packet[100] = { 0xFF, 0xFF, servoID, cmd, checkSum1, checkSum2, };

	checkSum1 = CheckSum1(packet, packetSize);
	checkSum2 = CheckSum2(checkSum1);

	uartTx(packet, packetSize);

}

/*********************** end *************************/

Herkulex::Herkulex() {
#ifdef DEBUG
	std::cout << "Herkulex::Herkulex()" << std::endl;
#endif
}

Herkulex::~Herkulex() {
#ifdef DEBUG
	std::cout << "Herkulex::~Herkulex()" << std::endl;
#endif
}

int32_t Herkulex::Initialize(OPRoS::Property props) {
#ifdef DEBUG
	std::cout << "Herkulex::Initialize()" << std::endl;
#endif
	uart_init("/dev/ttyS1", 115200);

	Reboot(BROADCAST_ID);
	usleep(200000);

	mProps = props;

	return API_SUCCESS;
}

int32_t Herkulex::Finalize(void) {
#ifdef DEBUG
	std::cout << "Herkulex::Finalize()" << std::endl;
#endif

	TorquePolicy(BROADCAST_ID, TORQUE_FREE);

	uart_close();

	return API_SUCCESS;
}

int32_t Herkulex::Enable(void) {
#ifdef DEBUG
	std::cout << "Herkulex::Enable()" << std::endl;
#endif

	TorquePolicy(BROADCAST_ID, TORQUE_ON);

	return API_SUCCESS;
}

int32_t Herkulex::Disable(void) {
#ifdef DEBUG
	std::cout << "Herkulex::Disable()" << std::endl;
#endif

	TorquePolicy(BROADCAST_ID, TORQUE_BREAK);

	return API_SUCCESS;
}

int32_t Herkulex::SetProperty(OPRoS::Property props) {
#ifdef DEBUG
	std::cout << "Herkulex::SetProperty()" << std::endl;
#endif

	mProps = props;

	return API_SUCCESS;
}

int32_t Herkulex::GetProperty(OPRoS::Property &props) {
#ifdef DEBUG
	std::cout << "Herkulex::GetProperty()" << std::endl;
#endif

	props = mProps;
	props.status = OPROS_SUCCESS;

	return API_SUCCESS;
}

int32_t Herkulex::RunHoming(void) {
#ifdef DEBUG
	std::cout << "Herkulex::RunHoming()" << std::endl;
#endif

//	ClearError(BROADCAST_ID);
//	usleep(20000);
//	TorquePolicy(BROADCAST_ID, TORQUE_ON);

	std::vector<OPRoS::Float64> pos(16, 0);
	std::vector<uint32_t> playtime(16, 1000);

	pos[0].data = atoi(mProps.GetValue("Angle0").c_str());
	pos[1].data = atoi(mProps.GetValue("Angle1").c_str());
	pos[3].data = atoi(mProps.GetValue("Angle3").c_str());
	pos[4].data = atoi(mProps.GetValue("Angle4").c_str());

	SetAngle(pos, playtime);

	return API_SUCCESS;
}

int32_t Herkulex::SetTorque(std::vector<OPRoS::Float64> torque) {
#ifdef DEBUG
	std::cout << "Herkulex::SetTorque()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::GetTorque(std::vector<OPRoS::Float64> &torque) {
#ifdef DEBUG
	std::cout << "Herkulex::GetTorque()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::SetVelocity(std::vector<OPRoS::Float64> velocity) {
#ifdef DEBUG
	std::cout << "Herkulex::SetVelocity()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::GetVelocity(std::vector<OPRoS::Float64> &velocity) {
#ifdef DEBUG
	std::cout << "Herkulex::GetVelocity()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::SetPosition(std::vector<OPRoS::Float64> position,
		std::vector<uint32_t> time) {
#ifdef DEBUG
	std::cout << "Herkulex::SetPosition()" << std::endl;
#endif

	SetAngle(position, time);

	return API_SUCCESS;
}

int32_t Herkulex::GetPosition(std::vector<OPRoS::Float64> &position) {
#ifdef DEBUG
	std::cout << "Herkulex::GetPosition()" << std::endl;
#endif

	return API_SUCCESS;
}

int32_t Herkulex::SetAngle(std::vector<OPRoS::Float64> angle,
		std::vector<uint32_t> time) {
#ifdef DEBUG
	std::cout << "Herkulex::SetAngle()" << std::endl;
#endif

	int pos[16] = { 0, };
	int playTime[16] = { 0, };
	uint8_t led[16] = { 0, };

	for (int i = 0; i < 16; ++i) {
		if (angle[i].data > 159.8f)
			pos[i] = 1002;
		else if (angle[i].data < -159.8f)
			pos[i] = 21;
		else
			pos[i] = (int) (angle[i].data * 3.07f) + 512;

		playTime[i] = time[i];
		led[i] = LED_NONE;
	}

	Moving(pos, playTime, led);

	return API_SUCCESS;
}
//int32_t Herkulex::SetAngle(std::vector<OPRoS::Float64> angle, std::vector<uint32_t> time,
//		std::vector<uint8_t> color) {
//#ifdef DEBUG
//	std::cout << "Herkulex::SetAngle() with color" << std::endl;
//#endif
//
//	int pos[16] = { 0, };
//	int playTime[16] = { 0, };
//	uint8_t led[16] = { 0, };
//
//	for (int i = 0; i < 16; ++i) {
//		if (angle[i].data > 159.8f)
//			pos[i] = 1002;
//		else if (angle[i].data < -159.8f)
//			pos[i] = 21;
//		else
//			pos[i] = (int) (angle[i].data * 3.07f) + 512;
//
//		playTime[i] = time[i];
//		led[i] = color[i];
//	}
//
//	Moving(pos, playTime, led);
//
//	return API_SUCCESS;
//}



#if defined(WIN32)
extern "C"
{
	__declspec(dllexport) OprosDevice *GetAPI();
}

OprosDevice *GetAPI()
{
	return new Herkulex();
}
#else
extern "C" {
OprosDevice *GetAPI();
}

OprosDevice *GetAPI() {
	return new Herkulex();
}
#endif


#ifndef HERKULEX_H_
#define HERKULEX_H_

#include <device/Manipulator.h>
#include <termios.h>

#define DEBUG

#define DATA_SIZE	 30		// buffer for input data
#define DATA_MOVE  	 100		// max 10 servos <---- change this for more servos!
#define TIME_OUT     5   	//timeout serial communication

// SERVO HERKULEX COMMAND(request packet) - See Manual p40
#define EEP_WRITE    0x01 	//Rom write
#define EEP_READ     0x02 	//Rom read
#define RAM_WRITE	 0x03 	//Ram write
#define RAM_READ	 0x04 	//Ram read
#define I_JOG		 0x05 	//Write n servo with different timing
#define S_JOG		 0x06 	//Write n servo with same time
#define STAT	 	 0x07 	//Read error
#define ROLLBACK	 0x08 	//Back to factory value
#define REBOOT	 	 0x09 	//Reboot

// Torque
#define TORQUE_RAM_ADDR 0x34

#define TORQUE_ON 0x60	// enable to move, but only by packet
#define TORQUE_BREAK 0x40	// disable to move
#define TORQUE_FREE 0x00		// enable to move freely

// HERKULEX LED(bit 1:ON, 0:OFF) - See Manual p29
#define LED_RAM_ADDR 0x35

#define LED_NONE 0x00
#define LED_GREEN 0x01
#define LED_BLUE 0x02
#define LED_CYAN 0x03
#define LED_RED 0x04
#define LED_GREEN2 0x05
#define LED_PINK 0x06
#define LED_WHITE 0x07

// HERKULEX STATUS ERROR - See Manual p39
#define STATUS_ERROR_ADDR 0x30

#define STATUS_OK 0x00
#define ERROR_INPUT_VOLTAGE 0x01
#define ERROR_POS_LIMIT 0x02
#define ERROR_TEMPERATURE_LIMIT 0x04
#define ERROR_INVALID_PKT 0x08
#define ERROR_OVERLOAD 0x10
#define ERROR_DRIVER_FAULT 0x20
#define ERROR_EEPREG_DISTORT 0x40

// HERKULEX Broadcast Servo ID
#define BROADCAST_ID 0xFE

#define SERVO_NUMBER 16

class Herkulex: public Manipulator {
private:
	int fd;
	termios tio;

	OPRoS::Property mProps;

private:

private:
	//Herkulex Protocol
	void TorquePolicy(uint8_t servoID, uint8_t state);
	void ClearError(uint8_t servoID);
	void GetError(uint8_t servoID);
	void Reboot(uint8_t servoID);

	uint8_t CheckSum1(uint8_t* packet, int size);uint8_t CheckSum2(
	uint8_t checkSum1);

private:
	void SendCommand(uint8_t servoID, uint8_t cmd, uint8_t* optionalData,
			int dataSize);

	int uart_init(const char* portName, int baudrate);
	// default baudrate : 115200
	void uart_close();
	int uartTx(const unsigned char* data, int size);
	int uartRx(unsigned char* data, int size);
	int setBaudrate(int baudrate);

	void littleEndian(uint8_t* lsb, uint8_t* msb, const int data);
	void Moving(const int* goal, const int* playTime, const uint8_t* color);
	void MovingSync(const int* goal, const uint8_t* color, const int playTime);
	void MovingOne(const int* goal, const int playTime, const uint8_t* color);
	// playTime : 0 ~ 2856 [msce]
	void SetLED(const uint8_t servoID, const uint8_t color);

public:
	Herkulex();
	~Herkulex();
public:
	int32_t Initialize(OPRoS::Property props);
	int32_t Finalize(void);int32_t Enable(void);
	int32_t Disable(void);
	int32_t SetProperty(OPRoS::Property props);
	int32_t RunHoming(void);int32_t SetTorque(std::vector<OPRoS::Float64> torque);
	int32_t GetTorque(std::vector<OPRoS::Float64> &torque);
	int32_t SetVelocity(std::vector<OPRoS::Float64> velocity);
	int32_t GetVelocity(std::vector<OPRoS::Float64> &velocity);
	int32_t SetPosition(std::vector<OPRoS::Float64> position, std::vector<uint32_t> time);
	int32_t GetPosition(std::vector<OPRoS::Float64> &position);

public:
	int32_t SetAngle(std::vector<OPRoS::Float64> angle, std::vector<uint32_t> time);
	int32_t SetAngle(std::vector<OPRoS::Float64> angle, std::vector<uint32_t> time, std::vector<uint8_t> color);
	int32_t SetPosition(std::vector<OPRoS::Float64> position, std::vector<uint32_t> time, std::vector<uint8_t> color);
	int32_t SetLED(std::vector<uint8_t> color);
};

#endif /* HERKULEX_H_ */


#include <stdint.h>
#include <chrono>
#include <thread>
#include <cctype> 

#include "OHandSerialAPI.h"

using namespace std;

#define PORT_TYPE_UART 1
#define PORT_TYPE_CAN 2

#define PORT_TYPE PORT_TYPE_CAN


#if (PORT_TYPE == PORT_TYPE_UART)
#include "uart.h"
#define PORT_Init UART_Init
#define PORT_DeInit UART_DeInit
#define PORT_SendData UART_SendData
#define PORT_RecvData UART_RecvData
#define BAUD_RATE 115200
#elif  (PORT_TYPE == PORT_TYPE_CAN)
#include "can.h"
#define PORT_Init CAN_Init
#define PORT_DeInit CAN_DeInit
#define PORT_SendData CAN_SendData
#define PORT_RecvData CAN_RecvData
#define BAUD_RATE 1000000
#else
#error Invalid PORT_RTPE
#endif

#ifdef _MSC_VER
#include <conio.h>
#define kbhit _kbhit
#define getch _getch
#endif

#ifndef _MSC_VER
// Ubuntu kbhit() function
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
int kbhit() {
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF) {
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

// Ubuntu getch() function
int getch() {
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
}
#endif


//----------------OHand-----------------//
#define ADDRESS_MASTER 0x01
#define ADDRESS_HAND 0x02

// #define SET_PID   // Uncomment this if you want to set PID

#define HAS_THUMB_ROOT_MOTOR

#define NUM_FINGERS (5)

#define FORCE_TARGET_VALUE 200

#define NUM_MOTORS (6)
#define THUMB_ROOT_ID (5)

#ifdef SET_PID
static const float _pidGains[][4] = {
	{250.00, 2.00, 250.00, 1.00},
	{250.00, 2.00, 250.00, 1.00},
	{250.00, 2.00, 250.00, 1.00},
	{250.00, 2.00, 250.00, 1.00},
	{250.00, 2.00, 250.00, 1.00},
#ifdef HAS_THUMB_ROOT_MOTOR
	{250.00, 2.00, 250.00, 1.00}
#endif
};
#endif

static uint8_t force_entries[NUM_MOTORS] = { (7 * 5),(12 * 5),(12 * 5),(12 * 5),(8 * 4),(11 * 5) };

//----------------global variable-----------------//
void* port = NULL;

void* hand_ctx = NULL;

uint8_t err, remote_err = 0;
//----------------system functions-----------------//
uint32_t millis();
void delay(uint32_t millisecondsToWait);

void setup();
void loop();
void force_control(uint8_t finger_id, uint16_t force_value);

int main(int argc, char* argv[])
{
	if (argc <= 1)
	{
		printf("Usage:\n");
		printf("%s <port_name>\n\n", argv[0]);
		exit(-1);
	}

	const char* port_name = argv[1];

	printf("Use port: '%s'\n", port_name);

	port = PORT_Init(port_name, BAUD_RATE);


	setup();

	HAND_ResetForce(hand_ctx, ADDRESS_HAND, &remote_err);

	while (true)
	{
		loop();
	}

	PORT_DeInit();

	printf("program exited with success.\n");
	exit(0);
}

// -------------------------------------------------------
// user functions for OHand

uint32_t millis()
{
	chrono::time_point<chrono::system_clock, chrono::milliseconds> tp = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now());
	auto tmp = chrono::duration_cast<chrono::milliseconds>(tp.time_since_epoch());
	time_t timestamp = tmp.count();
	static uint64_t startTime = (uint64_t)timestamp;
	return (uint32_t)(timestamp - startTime);
}


void delay(uint32_t millisecondsToWait)
{
	this_thread::sleep_for(chrono::milliseconds(millisecondsToWait));
}


void setup()
{
	uint8_t err, remote_err;

	printf("Waiting 4 seconds for OHand ready...\n");

	delay(4000); // Let OHand boot

	printf("Begin.\n");

	hand_ctx = HAND_CreateContext(port, HAND_PROTOCOL_UART, ADDRESS_MASTER, PORT_SendData, PORT_RecvData); // For non-interrupt receive mode, specify receive function.
	HAND_SetCommandTimeOut(hand_ctx, 255);
	HAND_SetTimerFunction(millis, delay);

	do
	{
		uint8_t major_ver, minor_ver;
		uint16_t revision;

		err = HAND_GetFirmwareVersion(hand_ctx, ADDRESS_HAND, &major_ver, &minor_ver, &revision, &remote_err);
		printf("HAND_GetFirmwareVersion() returned 0x%02x\n", err);

		if (err == HAND_RESP_SUCCESS)
		{
			printf("major_ver: 0x%02d\n", major_ver);

			//if (major_ver < 3)
			//{
			//  printf("OHand firmware version 3.x required, current is '0x%02x'\n", major_ver);
			//  exit(-1);
			//}
		}
		else
		{
			delay(100);
		}
	} while (err != HAND_RESP_SUCCESS);

	// Open fingers
	uint16_t raw_open_pos[NUM_MOTORS] = { 0, 0, 0, 0, 0, 0 };
	uint8_t raw_speed[NUM_MOTORS] = {255, 255, 255, 255, 255, 255};

	err = HAND_SetFingerPosAll(hand_ctx, ADDRESS_HAND, raw_open_pos, raw_speed, NUM_MOTORS, &remote_err);
	if (err != HAND_RESP_SUCCESS)
	{
		printf("HAND_SetFingerPosAll() returned 0x%02x\n", err);
	}

#ifdef SET_PID
	// Set PID
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		do
		{
			err = HAND_SetFingerPID(hand_ctx, ADDRESS_HAND, i, _pidGains[i][0], _pidGains[i][1], _pidGains[i][2], _pidGains[i][3], &remote_err);

			if (err != HAND_RESP_SUCCESS)
			{
				delay(1000);
				printf("HAND_SetFingerPID() returned 0x%02x\n", err);
			}
		} while (err != HAND_RESP_SUCCESS);
	}
#endif
}

void force_control(uint8_t finger_id, uint16_t force_value){
	err = HAND_SetFingerForceTarget(hand_ctx, ADDRESS_HAND, finger_id, force_value, &remote_err);
	if (err != HAND_RESP_SUCCESS){
		printf("HAND_SetFingerForceTarget with finger id: %d returned failed, result: 0x%02x\n", finger_id, err);
	}

	if (force_value == 0){
		err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, finger_id, 0, 255, &remote_err);
		if (err != HAND_RESP_SUCCESS){
		    printf("HAND_SetFingerForceTarget with finger id: %d returned failed, result: 0x%02x\n", finger_id, err);
	    }
	}
}

void loop()
{
	if (kbhit())
	{
		int key = getch();

		if (key != EOF)
		{
			// Distinguish between ordinary characters and special keys (such as arrow keys, function keys)
			if (key == 0 || key == 0xE0)
			{
				// Handle extended keys (such as arrow keys, F1-F12, etc.)
				int extended_key = getch();
				printf("Pressed special key: 0x%02X (extended: 0x%02X)\n", key, extended_key);
			}
			else
			{
				// Handle ordinary ASCII characters
				if (isprint(key))
				{
					printf("Pressed key: '%c' (ASCII: %d, Hex: 0x%02X)\n", key, key, key);
				}
				else // Non-printable characters (such as carriage return, backspace, ESC, etc.)
				{
					printf("Pressed control key: (ASCII: %d, Hex: 0x%02X)\n", key, key);
				}
			}

			switch (key)
			{
				// start force control
				case 'q':
				case 'Q':
					force_control(0, FORCE_TARGET_VALUE);
                    break;
				case 'w':
				case 'W':
					force_control(1, FORCE_TARGET_VALUE);
                    break;
				case 'e':
				case 'E':
					force_control(2, FORCE_TARGET_VALUE);
                    break;
				case 'r':
				case 'R':
					force_control(3, FORCE_TARGET_VALUE);
                    break;
				case 't':
				case 'T':
					force_control(4, FORCE_TARGET_VALUE);
                    break;
				
				// stop force control
				case 'a':
				case 'A':
					force_control(0, 0);
                    break;
				case 's':
				case 'S':
					force_control(1, 0);
                    break;
				case 'd':
				case 'D':
					force_control(2, 0);
					break;
				case 'f':
				case 'F':
					force_control(3, 0);
					break;
				case 'g':
				case 'G':
					force_control(4, 0);
					break;
			}
		}
	}
	
	// get finger force
	{
		uint8_t force[MAX_FORCE_ENTRIES] = { 0 };
		int sum_force[NUM_MOTORS] = { 0 };
		uint8_t force_entry_count;

		for (int finger_index = 0; finger_index < NUM_MOTORS; finger_index++) {
			force_entry_count = force_entries[finger_index];
			int value = 0;

			err = HAND_GetFingerForce(hand_ctx, ADDRESS_HAND, finger_index, &force_entry_count, force, &remote_err);

			if (err == HAND_RESP_SUCCESS) {
				for(int index = 0; index < force_entry_count; index++){
					value += force[index];
				}

				sum_force[finger_index] = value;
			}
			else{
				printf("ERROR! HAND_GetFingerForce returned: 0x%02x\n", err);
			}
		}

		printf("Thumb force: %d, Index force: %d, Middle force: %d, Ring force: %d, Little force: %d, Palm force: %d\n", 
			sum_force[0], sum_force[1], sum_force[2], sum_force[3], sum_force[4], sum_force[5]);
	}

}

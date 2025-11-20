
#include <stdint.h>
#include <chrono>
#include <thread>

#include "serial/serial.h"

#include "OHandSerialAPI.h"

using namespace std;

#define PORT_TYPE_UART 1
#define PORT_TYPE_CAN 2

#define PORT_TYPE PORT_TYPE_UART


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



//----------------OHand-----------------//
#define ADDRESS_MASTER 0x01
#define ADDRESS_HAND 0x02

// #define SET_PID   // Uncomment this if you want to set PID

#define HAS_THUMB_ROOT_MOTOR

#define NUM_FINGERS (5)

#ifdef HAS_THUMB_ROOT_MOTOR
#define NUM_MOTORS (6)
#define THUMB_ROOT_ID (5)
#else
#define NUM_MOTORS (NUM_FINGERS)
#endif

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

//----------------global variable-----------------//
void* port = NULL;

void* hand_ctx = NULL;

//----------------system functions-----------------//
uint32_t millis();
void delay(uint32_t millisecondsToWait);
void sendDataUART(uint8_t addr, uint8_t *data, uint8_t len, void *hand_ctx);
void recvDataUART(void *hand_ctx);

void setup();
void loop();

int main(int argc, char *argv[])
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

  // Open thumb
  err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, 0, 0, 255, &remote_err);
  if (err != HAND_RESP_SUCCESS)
  {
    printf("HAND_SetFingerPos() returned 0x%02x\n", err);
  }
  delay(1000);

  // Open others
  for (int i = 1; i < NUM_FINGERS; i++)
  {
    err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, i, 0, 255, &remote_err);
    if (err != HAND_RESP_SUCCESS)
    {
      printf("HAND_SetFingerPos() returned 0x%02x\n", err);
    }
  }
  delay(1000);

#ifdef HAS_THUMB_ROOT_MOTOR
  // Open thumb root
  err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, THUMB_ROOT_ID, 0, 255, &remote_err);
  if (err != HAND_RESP_SUCCESS)
  {
    printf("HAND_SetFingerPos() returned 0x%02x\n", err);
  }
  delay(1000); 
#endif
}

void loop()
{
  uint8_t i;
  uint8_t err, remote_err;

  // Close thumb
  err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, 0, 65535, 255, &remote_err);
  if (err != HAND_RESP_SUCCESS)
  {
    printf("HAND_SetFingerPos() returned 0x%02x\n", err);
  }
  delay(1500);

  // Open thumb
  err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, 0, 0, 255, &remote_err);
  if (err != HAND_RESP_SUCCESS)
  {
    printf("HAND_SetFingerPos() returned 0x%02x\n", err);
  }
  delay(1500);

  // Close other fingers one by one
  for (i = 1; i < NUM_FINGERS; i++)
  {
    err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, i, 65535, 255, &remote_err);
    if (err != HAND_RESP_SUCCESS)
    {
      printf("HAND_SetFingerPos() returned 0x%02x\n", err);
    }
    delay(200);
  }

  delay(1500);

  // Open other fingers one by one
  for (i = NUM_FINGERS - 1; i > 0; i--)
  {
    err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, i, 0, 255, &remote_err);
    if (err != HAND_RESP_SUCCESS)
    {
      printf("HAND_SetFingerPos() returned 0x%02x\n", err);
    }
    delay(200);
  }

  delay(1500);

#ifdef HAS_THUMB_ROOT_MOTOR
  // Close thumb root
  err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, THUMB_ROOT_ID, 65535, 255, &remote_err);
  if (err != HAND_RESP_SUCCESS)
  {
    printf("HAND_SetFingerPos() returned 0x%02x\n", err);
  }
  delay(1500);

  // Open thumb root
  err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, THUMB_ROOT_ID, 0, 255, &remote_err);
  if (err != HAND_RESP_SUCCESS)
  {
    printf("HAND_SetFingerPos() returned 0x%02x\n", err);
  }
  delay(1500);
#endif
}

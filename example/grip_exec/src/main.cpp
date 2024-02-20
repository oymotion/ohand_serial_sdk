
#include <stdint.h>
#include <chrono>
#include <thread>

#include "serial/serial.h"

#include "OHandGripExec.h"

using namespace std;

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
    {100.00, 2.00, 250.00, 1.00},
    {100.00, 2.00, 250.00, 0.25},
    {100.00, 2.00, 250.00, 0.25},
    {100.00, 2.00, 250.00, 0.25},
    {100.00, 2.00, 250.00, 0.25},
#ifdef HAS_THUMB_ROOT_MOTOR
    {100.00, 2.00, 250.00, 1.00}
#endif
};
#endif

//----------------global variables-----------------//
serial::Serial *serial_port = NULL;

uint8_t all_grips[] = {
    GRIP_FIST,
    GRIP_MOUSE,
    GRIP_KEY,
    GRIP_POINT,
    GRIP_COLUMN,
    GRIP_PALM,
    GRIP_SALUTE,
    GRIP_CHOPSTICK,
    GRIP_POWER,
    GRIP_GRASP,
    GRIP_LIFT,
    GRIP_PLATE,
    GRIP_BUCKLE,
    GRIP_PINCH_IC,
    GRIP_PINCH_IO,
    GRIP_PINCH_TC,
    GRIP_PINCH_TO,
    GRIP_PINCH_ITC,
    GRIP_TRIPOD_IC,
    GRIP_TRIPOD_IO,
    GRIP_TRIPOD_TC,
    GRIP_TRIPOD_TO,
    GRIP_TRIPOD_ITC,
    GRIP_GUN,
    GRIP_LOVE,
    GRIP_SWEAR,
    GRIP_VICTORY,
    GRIP_SIX,
};

//----------------system functions-----------------//
extern "C"
{
  uint32_t millis();
  void delay(uint32_t millisecondsToWait);
  void sendDataUART(uint8_t addr, uint8_t *data, uint8_t len);
  void recvDataUART();
  void errorHandler(uint8_t node_addr, uint8_t err);
}

void setup();

int main(int argc, char *argv[])
{
  if (argc <= 1)
  {
    printf("Usage:\n");
    printf("%s <serial_port_name>\n\n", argv[0]);
    exit(-1);
  }

  const char *serial_port_name = argv[1];

  printf("serial port name: '%s'\n", serial_port_name);

  serial_port = new serial::Serial(serial_port_name, 115200, serial::Timeout::simpleTimeout(300));

  printf("serial port '%s' open? %s.\n", serial_port_name, (serial_port->isOpen() ? "Yes" : "No"));

  if (!serial_port->isOpen())
  {
    printf("error: serial could not be opened \n");
    delete serial_port;

    exit(-1);
  }

  setup();

  // Execute grips one by one
  for (int i = 0; i < sizeof(all_grips) / sizeof(all_grips[0]); i++)
  {
    uint8_t finger_errors[NUM_MOTORS] = {
        0,
    };

    HAND_ExecGrip(ADDRESS_HAND, (uint8_t)i, finger_errors);

    // TODO: Process errors
    printf("finger_errors: {0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x}\n",
           finger_errors[0], finger_errors[1], finger_errors[2],
           finger_errors[3], finger_errors[4], finger_errors[5]);
  }

  serial_port->close();
  delete serial_port;

  printf("program exit success \n");
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

void recvDataUART()
{
  uint8_t data = 0;

  while (serial_port->available() != 0)
  {
    serial_port->read(&data, 1);
    // printf("receive : data = 0x%x \n" , data);
    HAND_OnData(data);
  }
}

void sendDataUART(uint8_t addr, uint8_t *data, uint8_t size)
{
  // printf("sendDataUART: byte size = %d \n" , len);
  vector<uint8_t> dataVector;

  for (int i = 0; i < size; i++)
  {
    // printf("data[%d] 0x%x \n" , i, data[i]);
    dataVector.push_back(data[i]);
  }

  serial_port->write(dataVector);
}

void errorHandler(uint8_t node_addr, uint8_t err)
{
  printf("NODE: 0x%02x, ERROR: 0x%02x\n", node_addr, err);
}

void setup()
{
  uint8_t err;

  printf("Waiting 4 seconds for OHand ready...\n");

  delay(4000); // Let OHand boot

  printf("Begin.\n");

  HAND_SetDataInterface(HAND_PROTOCOL_UART, ADDRESS_MASTER, sendDataUART, recvDataUART); // For non-interrupt receive mode, specify receive function.
  HAND_SetCommandTimeOut(255);
  HAND_SetTimerFunction(millis, delay);

  do
  {
    uint8_t major_ver, minor_ver;
    uint16_t revision;

    err = HAND_GetFirmwareVersion(ADDRESS_HAND, &major_ver, &minor_ver, &revision, errorHandler);
    printf("HAND_GetFirmwareVersion() returned 0x%02x\n", err);

    if (err == HAND_RESP_SUCCESS)
    {
      printf("major_ver: 0x%02d\n", major_ver);

      if (major_ver != FW_VER_MAJOR)
      {
        printf("oHand firmware version '0x%02x' not matches SDK's '0x%02x'\n", major_ver, FW_VER_MAJOR);

        exit(-1);
      }
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
      err = HAND_SetFingerPID(ADDRESS_HAND, i, _pidGains[i][0], _pidGains[i][1], _pidGains[i][2], _pidGains[i][3], errorHandler);

      if (err != HAND_RESP_SUCCESS)
      {
        delay(1000);
        printf("HAND_SetFingerPID() returned 0x%02x\n", err);
      }
    } while (err != HAND_RESP_SUCCESS);
  }
#endif

  // Open thumb
  err = HAND_FingerMove(ADDRESS_HAND, 0, 0, 255, errorHandler);
  if (err != HAND_RESP_SUCCESS)
  {
    printf("HAND_FingerMove() returned 0x%02x\n", err);
  }

  // Open others
  for (int i = 1; i < NUM_FINGERS; i++)
  {
    err = HAND_FingerMove(ADDRESS_HAND, i, 0, 255, errorHandler);
    if (err != HAND_RESP_SUCCESS)
    {
      printf("HAND_FingerMove() returned 0x%02x\n", err);
    }
  }

#ifdef HAS_THUMB_ROOT_MOTOR
  // Open thumb root
  err = HAND_FingerMove(ADDRESS_HAND, THUMB_ROOT_ID, 0, 255, errorHandler);
  if (err != HAND_RESP_SUCCESS)
  {
    printf("HAND_FingerMove() returned 0x%02x\n", err);
  }
#endif
}

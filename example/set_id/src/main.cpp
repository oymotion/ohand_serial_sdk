
#include <stdint.h>
#include <chrono>
#include <thread>

#include "serial/serial.h"

#include "OHandSerialAPI.h"

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

//----------------global variable-----------------//
serial::Serial *serial_port = NULL;

uint8_t node_ids[256] = {0};
uint8_t node_cnt = 0;

//----------------system functions-----------------//
uint32_t millis();
void delay(uint32_t millisecondsToWait);
void sendDataUART(uint8_t addr, uint8_t *data, uint8_t len);
void recvDataUART();
void errorHandler(uint8_t node_addr, uint8_t err);

void setup();
void loop();

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

  while (true)
  {
    loop();
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

  printf("Begin.\n\n");

  HAND_SetDataInterface(HAND_PROTOCOL_UART, ADDRESS_MASTER, sendDataUART, recvDataUART); // For non-interrupt receive mode, specify receive function.
  HAND_SetCommandTimeOut(255);
  HAND_SetTimerFunction(millis, delay);

  for (int i = 0; i < 256; i++)
  {
    uint8_t major_ver, minor_ver;
    uint16_t revision;

    printf("Trying node '0x%02x'...", i);

    err = HAND_GetFirmwareVersion((uint8_t)i, &major_ver, &minor_ver, &revision, errorHandler);
    // printf("HAND_GetFirmwareVersion() returned 0x%02x\n", err);

    if (err == HAND_RESP_SUCCESS)
    {
      printf("node '0x%02x' found, firmware version: %d.%d.%d\n", i, major_ver, minor_ver, revision);
      node_ids[node_cnt++] = (uint8_t)i;
    }
    else
    {
      printf("time out\n");
    }
  }

  printf("\nScan complete, %d nodes found.\n", node_cnt);
}

void loop()
{
  uint8_t err;

  int old_id, new_id;

  while (true)
  {
    printf("Nodes on bus:");

    for (int i = 0; i < node_cnt; i++)
    {
      printf("\t0x%02x\n", i);
    }

    printf("Please input old_id,new_id to modify ROH node ID:");
    scanf_s("%d,%d", &old_id, &new_id);

    err = HAND_SetID((uint8_t)old_id, (uint8_t)new_id, errorHandler);

    printf("Modify node id 0x%02x to 0x%02x, result=0x%02x.\n", old_id, new_id, err);
  }
}

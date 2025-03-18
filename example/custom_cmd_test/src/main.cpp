
#include <stdint.h>
#include <chrono>
#include <thread>

#include "serial/serial.h"

#include "OHandSerialAPI.h"

using namespace std;

#ifndef MAX
#define MAX(a, b) (((a) < (b))? (b) : (a))
#endif

#ifndef CLAMP
#define CLAMP(x, out_min, out_max) (((x) < (out_min))? (out_min) : (((x) > (out_max))? (out_max) : (x)))
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

#define THUMB_ROOT_POS_CNT 3

#define ANGLE_SCALE 100


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
serial::Serial *serial_port = NULL;
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
  std::vector<serial::PortInfo> ports;

  const char* serial_port_name;

  if (argc <= 1)
  {
    ports = serial::list_ports();

    if (ports.size() == 0)
    {
      printf("Port not found.");
      printf("Usage:\n");
      printf("%s <serial_port_name>\n\n", argv[0]);
      exit(-1);
    }
    else
    {
      serial_port_name = ports.at(ports.size() - 1).port.c_str();
    }
  }
  else
  {
    serial_port_name = argv[1];
  }

  printf("serial port name: '%s'\n", serial_port_name);


  try
  {
    serial_port = new serial::Serial(serial_port_name, 115200, serial::Timeout::simpleTimeout(300));
  }
  catch (serial::IOException e)
  {
    printf("Open serial failed: %s\n", e.what());
    exit(-1);
  }

  printf("serial port '%s' open? %s.\n", serial_port_name, (serial_port->isOpen() ? "Yes" : "No"));

  if (!serial_port->isOpen())
  {
    printf("error: serial could not be opened \n");
    delete serial_port;

    exit(-1);
  }

  setup();

  loop();

  serial_port->close();
  delete serial_port;

  printf("program exit with success\n");
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

void recvDataUART(void* hand_ctx)
{
  uint8_t data = 0;

  auto port = *(serial::Serial**)hand_ctx;

  while (port->available() != 0)
  {
    port->read(&data, 1);
    // printf("receive : data = 0x%x \n" , data);
    HAND_OnData(hand_ctx, data);
  }
}

void sendDataUART(uint8_t addr, uint8_t* data, uint8_t size, void* hand_ctx)
{
  // printf("sendDataUART: byte size = %d \n" , len);
  vector<uint8_t> dataVector;

  auto port = *(serial::Serial**)hand_ctx;

  for (int i = 0; i < size; i++)
  {
    // printf("data[%d] 0x%x \n" , i, data[i]);
    dataVector.push_back(data[i]);
  }

  port->write(dataVector);
}

void setup()
{
  uint8_t err, remote_err;

  //printf("Waiting 4 seconds for OHand ready...\n");
  //delay(4000); // Let OHand boot

  printf("Begin.\n");

  hand_ctx = HAND_CreateContext(serial_port, HAND_PROTOCOL_UART, ADDRESS_MASTER, sendDataUART, recvDataUART); // For non-interrupt receive mode, specify receive function.
  HAND_SetCommandTimeOut(hand_ctx, 255);
  HAND_SetTimerFunction(millis, delay);

  do
  {
    uint8_t major_ver, minor_ver;

    err = HAND_GetProtocolVersion(hand_ctx, ADDRESS_HAND, &major_ver, &minor_ver, &remote_err);
    printf("HAND_GetProtocolVersion() returned 0x%02x\n", err);

    if (err == HAND_RESP_SUCCESS)
    {
      printf("major_ver: 0x%02d\n", major_ver);

      if (major_ver != PROTOCOL_VERSION_MAJOR)
      {
        printf("OHand protocol version '0x%02x' not matches SDK's '0x%02x'\n", major_ver, PROTOCOL_VERSION_MAJOR);

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
      err = HAND_SetFingerPID(ADDRESS_HAND, i, _pidGains[i][0], _pidGains[i][1], _pidGains[i][2], _pidGains[i][3], &remote_err);

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

  // Open others
  for (int i = 1; i < NUM_FINGERS; i++)
  {
    err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, i, 0, 255, &remote_err);
    if (err != HAND_RESP_SUCCESS)
    {
      printf("HAND_SetFingerPos() returned 0x%02x\n", err);
    }
  }
}

void loop()
{
  uint8_t err, remote_err;

  const uint8_t MAX_WR_DATA_LEN
    = NUM_MOTORS * sizeof(uint16_t)   /* speed */
    + NUM_MOTORS * sizeof(uint16_t)   /* pos */
    + NUM_MOTORS * sizeof(uint16_t);  /* angle */

  const uint8_t MAX_RD_DATA_LEN
    = NUM_MOTORS * sizeof(uint16_t)  /* pos */
    + NUM_MOTORS * sizeof(uint16_t)  /* angle */
    + NUM_MOTORS * sizeof(uint16_t)  /* current */
    + NUM_MOTORS * sizeof(uint16_t)  /* force */
    + NUM_MOTORS * sizeof(uint8_t);  /* status */

  uint8_t data[MAX(MAX_WR_DATA_LEN, MAX_RD_DATA_LEN)];

  //int i = SUB_CMD_SET_SPEED | SUB_CMD_SET_POS;
  for (int i = 1; i < 256; i++)
  {
    uint8_t send_data_size;
    uint8_t recv_data_size;
    uint8_t data_flag;

    uint8_t* p_data = data;

    data_flag = (uint8_t)i;
    *p_data++ = data_flag;

    if ((data_flag & SUB_CMD_SET_POS) && (data_flag & SUB_CMD_SET_ANGLE))
    {
      printf("\nBoth SUB_CMD_SET_POS and SUB_CMD_SET_ANGLE enabled, skip.\n");
      continue;
    }

    printf("\ndata_flag: 0x%02x\n", data_flag);
    printf("SUB_CMD_SET_SPEED   %d\n", (data_flag & SUB_CMD_SET_SPEED)? 1 : 0);
    printf("SUB_CMD_SET_POS     %d\n", (data_flag & SUB_CMD_SET_POS) ? 1 : 0);
    printf("SUB_CMD_SET_ANGLE   %d\n", (data_flag & SUB_CMD_SET_ANGLE) ? 1 : 0);
    printf("SUB_CMD_GET_POS     %d\n", (data_flag & SUB_CMD_GET_POS) ? 1 : 0);
    printf("SUB_CMD_GET_ANGLE   %d\n", (data_flag & SUB_CMD_GET_ANGLE) ? 1 : 0);
    printf("SUB_CMD_GET_CURRENT %d\n", (data_flag & SUB_CMD_GET_CURRENT) ? 1 : 0);
    printf("SUB_CMD_GET_FORCE   %d\n", (data_flag & SUB_CMD_GET_FORCE) ? 1 : 0);
    printf("SUB_CMD_GET_STATUS  %d\n", (data_flag & SUB_CMD_GET_STATUS) ? 1 : 0);


    //
    // Compose data
    
    if (data_flag & SUB_CMD_SET_SPEED)
    {
      uint16_t speed[NUM_MOTORS] = { 1000, 1000, 1000, 1000, 1000 };

      printf("speed sent: ");

      for (int j = 0; j < NUM_MOTORS; j++)
      {
        *p_data++ = (uint8_t)speed[j];
        *p_data++ = (uint8_t)(speed[j] >> 8);

        printf("%5d ", speed[j]);
      }

      printf("\n");
    }

    if (data_flag & SUB_CMD_SET_POS)
    {
      uint16_t pos[NUM_MOTORS] = { 0, 32767, 32767, 32767, 32767, 0 };

      printf("pos sent: ");

      for (int j = 0; j < NUM_MOTORS; j++)
      {
        *p_data++ = (uint8_t)pos[j];
        *p_data++ = (uint8_t)(pos[j] >> 8);

        printf("%5d ", pos[j]);
      }

      printf("\n");
    }

    if (data_flag & SUB_CMD_SET_ANGLE)
    {
      uint16_t angle[NUM_MOTORS] = {
        0 * ANGLE_SCALE,
        150 * ANGLE_SCALE,
        150 * ANGLE_SCALE,
        150 * ANGLE_SCALE,
        150 * ANGLE_SCALE,
        0 * ANGLE_SCALE};

      printf("angle sent: ");

      for (int j = 0; j < NUM_MOTORS; j++)
      {
        *p_data++ = (uint8_t)angle[j];
        *p_data++ = (uint8_t)(angle[j] >> 8);

        printf("%5d ", angle[j]);
      }

      printf("\n");
    }

    send_data_size = (uint8_t)(p_data - data);


    //
    // Send custom command
    
    recv_data_size = sizeof(data);

    const auto start{ std::chrono::steady_clock::now() };

    err = HAND_SetCustom(hand_ctx, ADDRESS_HAND, data, send_data_size, &recv_data_size, &remote_err);

    const auto end{ std::chrono::steady_clock::now() };
    const std::chrono::duration<double> elapsed_seconds{ end - start };
    printf("latency: %fms\n", elapsed_seconds.count() * 1000);

    if (err != HAND_RESP_SUCCESS)
    {
      printf("HAND_SetCustom() returned 0x%02x\n", err);
      delay(1000);
      return;
    }

    //
    // Parse data returned

    p_data = data;

    if (data_flag & SUB_CMD_GET_POS)
    {
      uint16_t pos[NUM_MOTORS];

      printf("pos received: ");

      for (int j = 0; j < NUM_MOTORS; j++)
      {
        pos[j] = *p_data++;
        pos[j] |= (*p_data++) << 8;

        printf("%5d ", pos[j]);
      }

      printf("\n");
    }

    if (data_flag & SUB_CMD_GET_ANGLE)
    {
      uint16_t angle[NUM_MOTORS];

      printf("angle received: ");

      for (int j = 0; j < NUM_MOTORS; j++)
      {
        angle[j] = *p_data++;
        angle[j] |= (*p_data++) << 8;

        printf("%5d ", angle[j]);
      }

      printf("\n");
    }

    if (data_flag & SUB_CMD_GET_CURRENT)
    {
      uint16_t current[NUM_MOTORS];

      printf("current received: ");

      for (int j = 0; j < NUM_MOTORS; j++)
      {
        current[j] = *p_data++;
        current[j] |= (*p_data++) << 8;

        printf("%5d ", current[j]);
      }

      printf("\n");
    }

    if (data_flag & SUB_CMD_GET_FORCE)
    {
      uint16_t force[NUM_MOTORS];

      printf("force received: ");

      for (int j = 0; j < NUM_MOTORS; j++)
      {
        force[j] = *p_data++;
        force[j] |= (*p_data++) << 8;

        printf("%5d ", force[j]);
      }

      printf("\n");
    }

    if (data_flag & SUB_CMD_GET_STATUS)
    {
      uint8_t status[NUM_MOTORS];

      printf("status received: ");

      for (int j = 0; j < NUM_MOTORS; j++)
      {
        status[j] = *p_data++;

        printf("%5d ", status[j]);
      }

      printf("\n");
    }


    delay(2000);

    // Open thumb
    err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, 0, 0, 255, &remote_err);
    if (err != HAND_RESP_SUCCESS)
    {
      printf("HAND_SetFingerPos() returned 0x%02x\n", err);
    }

    // Open others
    for (int i = 1; i < NUM_FINGERS; i++)
    {
      err = HAND_SetFingerPos(hand_ctx, ADDRESS_HAND, i, 0, 255, &remote_err);
      if (err != HAND_RESP_SUCCESS)
      {
        printf("HAND_SetFingerPos() returned 0x%02x\n", err);
      }
    }

    delay(2000);
  }

  delay(1500);
}

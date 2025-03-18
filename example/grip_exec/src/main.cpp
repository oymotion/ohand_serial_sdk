
#include <stdint.h>
#include <conio.h>
#include <chrono>
#include <thread>
#include <map>

#include "serial/serial.h"

#include "log.h"
#include "OHandGripExec.h"

using namespace std;


#ifdef _MSC_VER
#define kbhit _kbhit
#define getch _getch
#endif


//----------------OHand-----------------//
#define ADDRESS_MASTER 0x01
#define ADDRESS_HAND 0x02

#define SET_PID   // Uncomment this if you want to set PID

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

//----------------global variables-----------------//
serial::Serial* serial_port = NULL;

const uint8_t all_grips[] = {
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
    GRIP_SIX
};


map<int, const char*> grip_names =
{
  {GRIP_FIST, "GRIP_FIST"},
  {GRIP_MOUSE, "GRIP_MOUSE"},
  {GRIP_KEY, "GRIP_KEY"},
  {GRIP_POINT, "GRIP_POINT"},
  {GRIP_COLUMN, "GRIP_COLUMN"},
  {GRIP_PALM, "GRIP_PALM"},
  {GRIP_SALUTE, "GRIP_SALUTE"},
  {GRIP_CHOPSTICK, "GRIP_CHOPSTICK"},
  {GRIP_POWER, "GRIP_POWER"},
  {GRIP_GRASP, "GRIP_GRASP"},
  {GRIP_LIFT, "GRIP_LIFT"},
  {GRIP_PLATE, "GRIP_PLATE"},
  {GRIP_BUCKLE, "GRIP_BUCKLE"},
  {GRIP_PINCH_IC, "GRIP_PINCH_IC"},
  {GRIP_PINCH_IO, "GRIP_PINCH_IO"},
  {GRIP_PINCH_TC, "GRIP_PINCH_TC"},
  {GRIP_PINCH_TO, "GRIP_PINCH_TO"},
  {GRIP_PINCH_ITC, "GRIP_PINCH_ITC"},
  {GRIP_TRIPOD_IC, "GRIP_TRIPOD_IC"},
  {GRIP_TRIPOD_IO, "GRIP_TRIPOD_IO"},
  {GRIP_TRIPOD_TC, "GRIP_TRIPOD_TC"},
  {GRIP_TRIPOD_TO, "GRIP_TRIPOD_TO"},
  {GRIP_TRIPOD_ITC, "GRIP_TRIPOD_ITC"},
  {GRIP_GUN, "GRIP_GUN"},
  {GRIP_LOVE, "GRIP_LOVE"},
  {GRIP_SWEAR, "GRIP_SWEAR"},
  {GRIP_VICTORY, "GRIP_VICTORY"},
  {GRIP_SIX, "GRIP_SIX"}
};


void* hand_ctx = NULL;

//---------------- Functions for OHand API -----------------//
extern "C"
{
  uint32_t millis();
  void delay(uint32_t millisecondsToWait);
  void sendDataUART(uint8_t addr, uint8_t* data, uint8_t len, void *hand_ctx);
  void recvDataUART(void *hand_ctx);
}


void wait_key();
void setup();


int main(int argc, char* argv[])
{
  if (argc <= 1)
  {
    printf("Usage:\n");
    printf("%s <serial_port_name>\n\n", argv[0]);
    exit(-1);
  }

  const char* serial_port_name = argv[1];

  log_set_level(LOG_LVL_DEBUG);

  printf("Use serial port: '%s'\n", serial_port_name);

  try
  {
    serial_port = new serial::Serial(serial_port_name, 115200, serial::Timeout::simpleTimeout(300));
  }
  catch (serial::IOException e)
  {
    printf("Error: %s\n", e.what());
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

  wait_key();

  //while (true)
  {
    // Execute grips one by one
    for (int i = 0; i < sizeof(all_grips) / sizeof(all_grips[0]); i++)
    {
      uint8_t finger_errors[NUM_MOTORS] = {
          0,
      };

      printf("\n\n");
      printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
      printf("+ grip: %s\n", grip_names[all_grips[i]]);
      printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

      HAND_ExecGrip(hand_ctx, ADDRESS_HAND, all_grips[i], finger_errors);

      // TODO: Process errors
      printf("finger_errors: {0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x}\n",
        finger_errors[0], finger_errors[1], finger_errors[2],
        finger_errors[3], finger_errors[4], finger_errors[5]);

      // Wait for fingers being in position
      while (true)
      {
        uint16_t target_pos[NUM_MOTORS], current_pos[NUM_MOTORS];
        uint8_t err, remote_err;
        uint8_t motor_cnt = NUM_MOTORS;

        err = HAND_GetFingerPosAll(hand_ctx, ADDRESS_HAND, target_pos, current_pos, &motor_cnt, &remote_err);

        if (err != HAND_RESP_SUCCESS)
        {
          LOG_E("HAND_GetFingerPosAll(%d) returned 0x%02x\n", ADDRESS_HAND, err);
        }
        else
        {
          uint8_t finger;

          for (finger = 0; finger < NUM_FINGERS; finger++)
          {
            printf("HAND_FingersReachedPos(%d): tick=%d, finger=%d, current_pos=%d, target_pos=%d\n", ADDRESS_HAND, HAND_GetTick(), finger, current_pos[finger], target_pos[finger]);

            if (abs(current_pos[finger] - target_pos[finger]) >= 1024)
            {
              printf("  finger %d NOT in position, target=%d, current=%d\n", finger, target_pos[finger], current_pos[finger]);
              break;
            }
          }

          if (finger == NUM_FINGERS) break;
        }

        delay(100);
      }

      HAND_Beep(hand_ctx, ADDRESS_HAND, 200, NULL);

      printf("Grip '%s' finished.\n", grip_names[all_grips[i]]);
      delay(1000);
      //wait_key();

      HAND_ExecGrip(hand_ctx, ADDRESS_HAND, GRIP_RELAX, finger_errors);

      // TODO: Process errors
      printf("finger_errors: {0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x}\n",
        finger_errors[0], finger_errors[1], finger_errors[2],
        finger_errors[3], finger_errors[4], finger_errors[5]);

      delay(1000);
    }
  }

  serial_port->close();
  delete serial_port;

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


void wait_key()
{
  printf("Press any key to continue...\n");
  while (kbhit()) (void)getch();
  while (!kbhit()) delay(100);
}


void setup()
{
  uint8_t err, remote_err;

  printf("Waiting 4 seconds for OHand ready...\n");

  delay(4000); // Let OHand boot

  printf("Begin.\n");

  hand_ctx = HAND_CreateContext(serial_port, HAND_PROTOCOL_UART, ADDRESS_MASTER, sendDataUART, recvDataUART); // For non-interrupt receive mode, specify receive function.
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

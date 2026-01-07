#include "can.h"


#ifdef WIN32
#include <Windows.h>
#endif

#include <PCANBasic.h>
#include "OHandSerialAPI.h"
#include <stdio.h>

#define RUN 0
#define DEBUG 1

#define RUN_TYPE RUN

static TPCANHandle can_handle;


// Initialize
void* CAN_Init(const char* port_name, uint32_t baudrate)
{
  int port_num;
  TPCANBaudrate baud;

  if (sscanf(port_name, "%d", &port_num) != 1 || port_num < 1 || port_num > 16)
  {
    printf("Error: Invalid port number. It must be a number between 1 and 16.\n");
    return NULL;
  }

  can_handle = (TPCANHandle)(PCAN_USBBUS1 + (port_num - 1));

  switch (baudrate)
  {
  case 250000:
    baud = PCAN_BAUD_250K;
    break;

  case 500000:
    baud = PCAN_BAUD_500K;
    break;

  case 1000000:
    baud = PCAN_BAUD_1M;
    break;

  default:
    printf("Error: Unsupported baud rate. Must be 250000, 500000, or 1000000.\n");
    return NULL;
  }

  TPCANStatus status = CAN_Initialize(can_handle, baud, 0, 0, 0);

  if (status != PCAN_ERROR_OK)
  {
    printf("Error: CAN initialization failed. Error code: 0x%02X\n", status);
    return NULL;
  }

  return &can_handle;
}


// Release resources
void CAN_DeInit()
{
  CAN_Uninitialize(can_handle);
}


// Split-frame transmission, maximum 8 bytes payload per frame
void CAN_SendData(uint8_t addr, uint8_t* data, uint8_t len, void* hand_ctx)
{
  TPCANStatus status;
  TPCANMsg canMessage;

  canMessage.ID = addr;
  canMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;

  for (uint8_t i = 0; i < len; i += 8)
  {
    uint8_t currentSize = (len - i < 8) ? (len - i) : 8;
    canMessage.LEN = currentSize;

    for (uint8_t j = 0; j < currentSize; j++)
    {
      canMessage.DATA[j] = data[i + j];
    }

    status = CAN_Write(can_handle, &canMessage);

    if (status != PCAN_ERROR_OK)
    {
      printf("CAN_Write failed with error code 0x%02x\n", status);
      break;
    }

    if(RUN_TYPE == DEBUG){

      printf("Send frame: ID=0x%03X, LEN=%d, DATA=", canMessage.ID, canMessage.LEN);

      for (int i = 0; i < canMessage.LEN; i++){
        printf("%02X ", canMessage.DATA[i]);
      }

      printf("\n");
    }
  }
}


// Receive frame data
void CAN_RecvData(void* hand_ctx)
{
  TPCANStatus status;
  TPCANMsg message;
  TPCANTimestamp timestamp;
  TPCANHandle* handle = *(TPCANHandle**)hand_ctx;

  // Read one frame of data
  status = CAN_Read(*handle, &message, &timestamp);

  if (status == PCAN_ERROR_OK)
  {
    if(RUN_TYPE == DEBUG){

      // Print received CAN frame information
      printf("Receive frame: ID=0x%03X, LEN=%d, DATA=", message.ID, message.LEN);

      for (int i = 0; i < message.LEN; i++)
      {
        printf("%02X ", message.DATA[i]);
      }

      printf("\n");
    }
    
    // Process data
    for (int i = 0; i < message.LEN; i++)
    {
      HAND_OnData(hand_ctx, message.DATA[i]);
    }
  }
  else if (status == PCAN_ERROR_QRCVEMPTY)
  {
    // Receive queue is empty, exiting
    return;
  }
  else
  {
    // Other error
    printf("CAN_Read error. Error code:: 0x%02X\n", status);
    return;
  }
}


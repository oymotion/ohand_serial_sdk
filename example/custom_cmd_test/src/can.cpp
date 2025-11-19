#include "can.h"


#ifdef WIN32
#include <Windows.h>
#endif

#include <PCANBasic.h>
#include "OHandSerialAPI.h"
#include <stdio.h>
#include <vector>

#define RUN 0
#define DEBUG 1

#define RUNTYPE RUN

std::vector<TPCANHandle*> can_handles;


// 初始化
void* CAN_Init(const char* port_name, uint32_t baudrate)
{
    int port_num;
    TPCANBaudrate baud;
    TPCANHandle* can_handle = new TPCANHandle;

    if (sscanf(port_name, "%d", &port_num) != 1 || port_num < 1 || port_num > 16)
    {
        printf("err: Invalid port number. It must be a number between 1 and 16\n");
        return NULL;
    }

  *can_handle = (TPCANHandle)(PCAN_USBBUS1 + (port_num - 1));
  can_handles.push_back(can_handle);
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
    printf("Error: Unsupported baud rate. It must be 250000, 500000, or 1000000\n");
    return NULL;
  }

  TPCANStatus status = CAN_Initialize(*can_handle, baud, 0, 0, 0);

  if (status != PCAN_ERROR_OK)
  {
    printf("Error: CAN init failed,error code: 0x%02X\n", status);
    return NULL;
  }

  return can_handle;
}


// 释放资源
void CAN_DeInit()
{
    for (TPCANHandle* handle : can_handles)
    {
        if (handle != nullptr)
        {
            CAN_Uninitialize(*handle);
            delete handle;
            handle = nullptr;
        }
    }
    can_handles.clear();
    // Sleep(200);
}


// 分Frame发送，每个Frame的payload最大8字节
void CAN_SendData(uint8_t addr, uint8_t* data, uint8_t len, void* hand_ctx)
{
  if (hand_ctx == NULL) return;
  TPCANStatus status;
  TPCANMsg canMessage;
  TPCANHandle* can_handle = *(TPCANHandle**)hand_ctx;

  canMessage.ID = addr;

  for (uint8_t i = 0; i < len; i += 8)
  {
    uint8_t currentSize = (len - i < 8) ? (len - i) : 8;
    canMessage.LEN = currentSize;

    for (uint8_t j = 0; j < currentSize; j++)
    {
      canMessage.DATA[j] = data[i + j];
    }

    status = CAN_Write(*can_handle, &canMessage);

    if (status != PCAN_ERROR_OK)
    {
      printf("CAN_Write failed with error code 0x%02x\n", status);
      break;
    }
  }
}


// 接收Frame数据
void CAN_RecvData(void* hand_ctx)
{
  TPCANStatus status;
  TPCANMsg message;
  TPCANTimestamp timestamp;
  TPCANHandle* handle = *(TPCANHandle**)hand_ctx;

  // 读取一帧数据
  status = CAN_Read(*handle, &message, &timestamp);

  if (status == PCAN_ERROR_OK)
  {
#if RUNTYPE == DEBUG 
    // 打印接收到的CAN帧信息
    printf("Receive Frame: ID=0x%03X, LEN=%d, DATA=", message.ID, message.LEN);

    for (int i = 0; i < message.LEN; i++)
    {
        printf("%02X ", message.DATA[i]);
    }
    printf("\n");
#endif    

    // 处理数据
    for (int i = 0; i < message.LEN; i++)
    {
      HAND_OnData(hand_ctx, message.DATA[i]);
    }
  }
  else if (status == PCAN_ERROR_QRCVEMPTY)
  {
    // 接收队列为空，退出
    return;
  }
  else
  {
    // 其他错误
    printf("CAN_Read error, error code: 0x%02X\n", status);
    return;
  }
}


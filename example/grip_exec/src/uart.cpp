
#include "uart.h"

#include "serial/serial.h"

#include "OHandSerialAPI.h"


using namespace std;

static serial::Serial* serial_port = NULL;

void* UART_Init(const char* port_name, uint32_t baudrate)
{
  try
  {
    serial_port = new serial::Serial(port_name, baudrate, serial::Timeout::simpleTimeout(300));
  }
  catch (serial::IOException e)
  {
    printf("Error: %s\n", e.what());
    serial_port = NULL;
    return NULL;
  }

  printf("serial port '%s' open? %s.\n", port_name, (serial_port->isOpen() ? "Yes" : "No"));

  if (!serial_port->isOpen())
  {
    printf("error: serial could not be opened \n");
    delete serial_port;
    serial_port = NULL;

    return NULL;
  }

  return serial_port;
}


void UART_DeInit()
{
  delete serial_port;
  serial_port = NULL;
}


void UART_RecvData(void* hand_ctx)
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


void UART_SendData(uint8_t addr, uint8_t* data, uint8_t len, void* hand_ctx)
{
  // printf("sendDataUART: byte size = %d \n" , len);
  vector<uint8_t> dataVector;

  auto port = *(serial::Serial**)hand_ctx;

  for (int i = 0; i < len; i++)
  {
    // printf("data[%d] 0x%x \n" , i, data[i]);
    dataVector.push_back(data[i]);
  }

  port->write(dataVector);
}


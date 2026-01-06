

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  void* UART_Init(const char* port_name, uint32_t baudrate);

  void UART_DeInit();

  void UART_SendData(uint8_t addr, uint8_t* data, uint8_t len, void* hand_ctx);

  void UART_RecvData(void* hand_ctx);

#ifdef __cplusplus
}
#endif
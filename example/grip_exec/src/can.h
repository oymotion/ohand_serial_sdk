
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  void* CAN_Init(const char* port_name, uint32_t baudrate);

  void CAN_DeInit();

  void CAN_SendData(uint8_t addr, uint8_t* data, uint8_t len, void* hand_ctx);

  void CAN_RecvData(void* hand_ctx);

#ifdef __cplusplus
}
#endif
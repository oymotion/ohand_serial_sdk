
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
  
  /**
 * \brief 
 * \param port_name  : port_name
 * \param baudrate   : abit
 * \return canfd handle
 */
  void* CANFD_Init(const char* port_name, uint32_t baudrate);

  void CANFD_DeInit();

  void CANFD_SendData(uint8_t addr, uint8_t* data, uint8_t len, void* hand_ctx);

  void CANFD_RecvData(void* hand_ctx);

#ifdef __cplusplus
}
#endif

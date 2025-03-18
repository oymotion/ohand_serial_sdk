/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __OHAND_GRIP_EXEC_H__
#define __OHAND_GRIP_EXEC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

#include <stdint.h>

#include "OHandSerialAPI.h"

/* Exported constants --------------------------------------------------------*/

#define GRIP_FIST          0
#define GRIP_MOUSE         1
#define GRIP_KEY           2
#define GRIP_POINT         3
#define GRIP_COLUMN        4
#define GRIP_PALM          5
#define GRIP_SALUTE        6
#define GRIP_CHOPSTICK     7
#define GRIP_POWER         8
#define GRIP_GRASP         9
#define GRIP_LIFT         10
#define GRIP_PLATE        11
#define GRIP_BUCKLE       12
#define GRIP_PINCH_IC     13
#define GRIP_PINCH_IO     14
#define GRIP_PINCH_TC     15
#define GRIP_PINCH_TO     16
#define GRIP_PINCH_ITC    17
#define GRIP_TRIPOD_IC    18
#define GRIP_TRIPOD_IO    19
#define GRIP_TRIPOD_TC    20
#define GRIP_TRIPOD_TO    21
#define GRIP_TRIPOD_ITC   22

/* For demo only */
#define GRIP_GUN          23
#define GRIP_LOVE         24
#define GRIP_SWEAR        25
#define GRIP_VICTORY      26
#define GRIP_SIX          27

#define GRIP_NONE         ((uint8_t)-1)
#define GRIP_RELAX        ((uint8_t)-2)


/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Execute grip
  * @note
  * @param  hand_ctx: context of OHand
  * @param  hand_id: id of OHand
  * @param  new_grip: id of grip to be executed
  * @param  finger_errors: array to store errors for every finger, can be NULL
  * @retval None
  */
void HAND_ExecGrip(void *hand_ctx, uint8_t hand_id, uint8_t new_grip, uint8_t finger_errors[]);


#ifdef __cplusplus
}
#endif

#endif //__OHAND_GRIP_EXEC_H__

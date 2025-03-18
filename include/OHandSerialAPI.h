
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __HAND_SERIAL_API_H__
#define __HAND_SERIAL_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>


/* Exported types ------------------------------------------------------------*/

#ifndef NULL
#define NULL ((void*)0)
#endif

#ifndef BOOL
#define BOOL uint8_t
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


/* Exported constants --------------------------------------------------------*/

typedef enum
{
  HAND_PROTOCOL_UART,
  HAND_PROTOCOL_I2C
} HAND_PROTOCOL;



#define PROTOCOL_VERSION_MAJOR 3


// #define MAX_THUMB_ROOT_POS 3
// #define MAX_MOTOR_CNT 6
// #define MAX_FORCE_ENTRIES (12 * 5) /* Max force entries for one force sensor */


/*
 * OHand error codes
 */

/*
 * Error codes for remote peer, protocol part
 */
#define ERR_PROTOCOL_WRONG_LRC                          0x01


/*
 * Error codes for remote peer, command part
 */
#define ERR_COMMAND_INVALID                             0x11
#define ERR_COMMAND_INVALID_BYTE_COUNT                  0x12
#define ERR_COMMAND_INVALID_DATA                        0x13


/*
 * Error codes for remote peer, status part
 */
#define ERR_STATUS_INIT                                 0x21
#define ERR_STATUS_CALI                                 0x22
#define ERR_STATUS_STUCK                                0x23


/*
 * Error codes for remote peer, operation result part
 */
#define ERR_OP_FAILED                                   0x31
#define ERR_SAVE_FAILED                                 0x32


/*
 * API return values
 */
#define HAND_RESP_HAND_ERROR                            0xFF           /* device error, error call back will be called with OHand error codes listed above */

#define HAND_RESP_SUCCESS                               0x00
#define HAND_RESP_TIMER_FUNC_NOT_SET                    0x01           /* local error, timer function not set, call HAND_SetTimerFunction(...) first */
#define HAND_RESP_INVALID_CONTEXT                       0x02           /* local error, invalid context, NULL or send data function not set */
#define HAND_RESP_TIMEOUT                               0x03           /* local error, time out when waiting node response */
#define HAND_RESP_INVALID_OUT_BUFFER_SIZE               0x04           /* local error, out buffer size not matched to returned data */
#define HAND_RESP_UNMATCHED_ADDR                        0x05           /* local error, unmatched node id between returned and waiting */
#define HAND_RESP_UNMATCHED_CMD                         0x06           /* local error, unmatched command between returned and waiting */
#define HAND_RESP_DATA_SIZE_TOO_BIG                     0x07           /* local error, size of data to send exceeds the buffer size */
#define HAND_RESP_DATA_INVALID                          0x08           /* local error, data content invalid */


/* Sub-command for HAND_CMD_SET_CUSTOM */
#define SUB_CMD_SET_SPEED     (1 << 0)
#define SUB_CMD_SET_POS       (1 << 1)
#define SUB_CMD_SET_ANGLE     (1 << 2)
#define SUB_CMD_GET_POS       (1 << 3)
#define SUB_CMD_GET_ANGLE     (1 << 4)
#define SUB_CMD_GET_CURRENT   (1 << 5)
#define SUB_CMD_GET_FORCE     (1 << 6)
#define SUB_CMD_GET_STATUS    (1 << 7)


/* Exported macro ------------------------------------------------------------*/



/* Exported functions ------------------------------------------------------- */

/*
 * OHand Serial API
 */

/**
  * @brief  Create context and set master address, send data function and receive data function
  * @note   For none interrupt receive function, pass receive function pointer as parameter
  *         RecvDataImpl and call HAND_OnData(...) in receive function. For interrupt receive
  *         mode, pass NULL as parameter RecvDataImpl and call HAND_OnData(...) in ISR. Call
  *         this before using other get & set functions
  * @param  ctx_private_data: Context private data, e.g., address of serial port instance,
  *         port name string, etc. Resource should be available during runtime.
  * @param  protocol: HAND_PROTOCOL_UART or HAND_PROTOCOL_I2C
  * @param  address_master: master node address
  * @param  SendDataImpl: send data function pointer
  * @param  RecvDataImpl: receive data function pointer, NULL for interrupt receive mode
  * @retval Context
  */
void *HAND_CreateContext(
  const void *ctx_private_data,
  HAND_PROTOCOL protocol,
  uint8_t address_master,
  void (*SendDataImpl)(uint8_t addr, uint8_t *data, uint8_t size, void *ctx),
  void (*RecvDataImpl)(void *ctx)
);


/**
  * @brief  Free OHand context
  * @param  ctx: pointer to OHand context
  * @retval None
  */
void HAND_FreeContext(void *ctx);


/**
  * @brief  Set timer related function
  * @note   Call this before using get & set functions
  * @param  GetMilliSecondsImpl: pointer of function to get miliseconds
  * @param  DelayMilliSecondsImpl: pointer of function to delay miliseconds
  * @retval None
  */
void HAND_SetTimerFunction(uint32_t (*GetMilliSecondsImpl)(void), void (*DelayMilliSecondsImpl)(uint32_t ms));


/**
  * @brief  Get current tick of system in milliseconds 
  * @note   if HAND_SetTimerFunction(...) is not set, this will always return 0
  * @param  None
  * @retval None
  */
uint32_t HAND_GetTick(void);


/**
  * @brief  Set command timeout value
  * @note   Call this before using get & set functions. Default value is 255ms
  * @param  ctx: pointer to OHand context
  * @param  timeout: timeout in miliseconds
  * @param  DelayMilliSecondsImpl: pointer of function to delay miliseconds
  * @retval None
  */
void HAND_SetCommandTimeOut(void *ctx, uint16_t timeout);


/**
  * @brief  Feed data to internal parser
  * @note   Call this in data receive function, either RecvDataImpl or ISR
  * @param  ctx: pointer to OHand context
  * @param  data: data byte received
  * @retval None
  */
void HAND_OnData(void *ctx, uint8_t data);



/*
 * Gets
 */

/**
  * @brief  Get protocol version
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  major: pointer to uint8_t type var, when success, the major version of firmware will be stored
  * @param  minor: pointer to uint8_t type var, when success, the minor version of firmware will be stored
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetProtocolVersion(void *ctx, uint8_t hand_id, uint8_t *major, uint8_t *minor, uint8_t *remote_err);

/**
  * @brief  Get firmware version
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  major: pointer to uint8_t type var, when success, the major version of firmware will be stored
  * @param  minor: pointer to uint8_t type var, when success, the minor version of firmware will be stored
  * @param  revision: pointer to uint16_t type var, when success, the revision of firmware will be stored
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFirmwareVersion(void *ctx, uint8_t hand_id, uint8_t *major, uint8_t *minor, uint16_t *revision, uint8_t *remote_err);


/**
  * @brief  Get hardware version
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  hw_type: pointer to uint8_t type var, when success, the hardware type will be stored
  * @param  hw_ver: pointer to uint8_t type var, when success, the hardware version will be stored
  * @param  boot_version: pointer to uint16_t type var, when success, the major version will be stored in high byte,
  *                  while the minor ver in the low byte 
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetHardwareVersion(void *ctx, uint8_t hand_id, uint8_t *hw_type, uint8_t *hw_ver, uint16_t *boot_version, uint8_t *remote_err);


/**
  * @brief  Get calibration data
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  end_pos: address of array to put end_pos
  * @param  start_pos: address of array to put start_pos
  * @param  motor_cnt: motor count, i.e., array size of end_pos[] or start_pos[]
  * @param  thumb_root_pos: address of array to put thumb_root_pos
  * @param  thumb_root_pos_cnt: count of pre-defined thumb root pos
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetCaliData(void *ctx, uint8_t hand_id, uint16_t *end_pos, uint16_t *start_pos, uint8_t *motor_cnt, uint16_t *thumb_root_pos, uint8_t *thumb_root_pos_cnt, uint8_t *remote_err);


/**
  * @brief  Get PID gain
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  p: address of var to put PID gain p
  * @param  i: address of var to put PID gain i
  * @param  d: address of var to put PID gain d
  * @param  g: address of var to put PID gain g
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPID(void *ctx, uint8_t hand_id, uint8_t finger_id, float *p, float *i, float *d, float *g, uint8_t *remote_err);


/**
  * @brief  Get current limit of finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  current_limit: address of var to put current limit in mA
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerCurrentLimit(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t *current_limit, uint8_t *remote_err);


/**
  * @brief  Get current of finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  current: address of var to put current in mA
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerCurrent(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t *current, uint8_t *remote_err);


/**
  * @brief  Get force limit of finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  force_limit: address of var to put force limit in mN
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerForceLimit(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t *force_limit, uint8_t *remote_err);


/**
  * @brief  Get force of finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: actually sensor id
  * @param  force_entry_cnt: count of force entries
  * @param  force: address of var to put force, may be multi-point forces, or normal force, tangential force and dir
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerForce(void *ctx, uint8_t hand_id, uint8_t finger_id, uint8_t *force_entry_cnt, uint8_t *force, uint8_t *remote_err);


/**
  * @brief  Get absolute position limit of finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  low_limit: address of var to put low limit of finger position
  * @param  high_limit: address of var to put high limit of finger position
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPosLimit(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t *low_limit, uint16_t *high_limit, uint8_t *remote_err);


/**
  * @brief  Get absolute position of finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  target_pos: address of var to put raw target position, i.e., encoder value of finger
  * @param  current_pos: address of var to put raw current position, i.e., encoder value of finger
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPosAbs(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t *target_pos, uint16_t *current_pos, uint8_t *remote_err);


/**
  * @brief  Get logical position of finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  target_pos: address of var to put logical target position
  * @param  current_pos: address of var to put logical current position
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPos(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t *target_pos, uint16_t *current_pos, uint8_t *remote_err);


/**
  * @brief  Get angle of finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  target_angle: address of var to put logical target position, value = real angle * 100
  * @param  current_angle: address of var to put logical current position, value = real angle * 100
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerAngle(void *ctx, uint8_t hand_id, uint8_t finger_id, int16_t *target_angle, int16_t *current_angle, uint8_t *remote_err);


/**
  * @brief  Get position of thumb's root
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  raw_encoder: address of var to put raw encoder value of thumb's root, value range [0-65535]
  * @param  pos: address of var to put mapped position of thumb's root, value range [0-2], or 255 if invalid
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetThumbRootPos(void *ctx, uint8_t hand_id, uint16_t *raw_encoder, uint8_t *pos, uint8_t *remote_err);


/**
  * @brief  Get absolute position of all fingers
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  motor_cnt: number of motors
  * @param  target_pos: address of var to put raw target position, i.e., encoder value of finger
  * @param  current_pos: address of var to put raw current position, i.e., encoder value of finger
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPosAbsAll(void *ctx, uint8_t hand_id, uint16_t *target_pos, uint16_t *current_pos, uint8_t *motor_cnt, uint8_t *remote_err);


/**
  * @brief  Get logical position of all fingers
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  target_pos: address of var to put logical target position
  * @param  current_pos: address of var to put logical current position
  * @param  motor_cnt: number of motors
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPosAll(void *ctx, uint8_t hand_id, uint16_t *target_pos, uint16_t *current_pos, uint8_t *motor_cnt, uint8_t *remote_err);


/**
  * @brief  Get first joint angle of all fingers
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  target_angle: address of var to put target angle
  * @param  current_pos: address of var to put current angle
  * @param  motor_cnt: number of motors
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerAngleAll(void *ctx, uint8_t hand_id, int16_t *target_angle, int16_t *current_angle, uint8_t *motor_cnt, uint8_t *remote_err);



/**
  * @brief  Get self test level
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  self_test_on: address of var to put self-test level
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetSelfTestLevel(void *ctx, uint8_t hand_id, uint8_t *self_test_level, uint8_t *remote_err);


/**
  * @brief  Get beep switch
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  beep_on: address of var to put beep switch status
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetBeepSwitch(void *ctx, uint8_t hand_id, uint8_t *beep_switch, uint8_t *remote_err);


/**
  * @brief  Get button pressed count of OHand
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  pressed_cnt: button pressed count
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetButtonPressedCnt(void *ctx, uint8_t hand_id, uint8_t *pressed_cnt, uint8_t *remote_err);


/**
  * @brief  Get the 96 bits device UID
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  uid_w0: pointer to word 0 of UID
  * @param  uid_w1: pointer to word 1 of UID
  * @param  uid_w2: pointer to word 2 of UID
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetUID(void *ctx, uint8_t hand_id, uint32_t *uid_w0, uint32_t *uid_w1, uint32_t *uid_w2, uint8_t *remote_err);


/**
  * @brief  Get battery voltage
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  voltage: pointer to uint16_t type var, when success, the voltage in mv will be stored
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetBatteryVoltage(void *ctx, uint8_t hand_id, uint16_t *voltage, uint8_t *remote_err);


/**
  * @brief  Get usage stat
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  total_use_time: address of var to put total use time in seconds
  * @param  total_open_times: address of array to put total_open_times[5]
  * @param  motor_cnt: motor count, i.e., array size of total_use_time[] or total_open_times[]
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetUsageStat(void *ctx, uint8_t hand_id, uint32_t *total_use_time, uint32_t *total_open_times, uint8_t motor_cnt, uint8_t *remote_err);



/*
 * Sets
 */

 /**
  * @brief  Reset hand
  * @note   NO response, so return code must be timeout
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  mode: 0 boot to user code, 1 boot to DFU mode
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_Reset(void *ctx, uint8_t hand_id, uint8_t mode, uint8_t *remote_err);


 /**
  * @brief  Turn hand off
  * @note   NO response, so return code must be timeout
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_PowerOff(void *ctx, uint8_t hand_id, uint8_t *remote_err);


/**
  * @brief  Set hand id
  * @note   Need to reboot device after setting
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  new_id: new id for hand
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetID(void *ctx, uint8_t hand_id, uint8_t new_id, uint8_t *remote_err);


/**
  * @brief  Start calibration
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  key: key to enable calibration, to avoid mishandling
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_Calibrate(void *ctx, uint8_t hand_id, uint16_t key, uint8_t *remote_err);


/**
  * @brief  Set calibration data
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  end_pos: address of array to put end_pos
  * @param  start_pos: address of array to put start_pos
  * @param  motor_cnt: motor count, i.e., array size of end_pos[] or start_pos[]
  * @param  thumb_root_pos: address of array to put thumb_root_pos
  * @param  thumb_root_pos_cnt: count of pre-defined thumb root pos
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetCaliData(void *ctx, uint8_t hand_id, uint16_t *end_pos, uint16_t *start_pos, uint8_t motor_cnt, uint16_t *thumb_root_pos, uint8_t thumb_root_pos_cnt, uint8_t *remote_err);


/**
  * @brief  Set PID gain
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  p: PID gain p
  * @param  i: PID gain i
  * @param  d: PID gain d
  * @param  d: PID gain g
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPID(void *ctx, uint8_t hand_id, uint8_t finger_id, float p, float i, float d, float g, uint8_t *remote_err);


/**
  * @brief  Set finger current limit
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  current_limit: current limit in mA for finger
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerCurrentLimit(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t current_limit, uint8_t *remote_err);


/**
  * @brief  Set finger normal force limit
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  force_limit: force limit in mN for finger
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerForceLimit(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t force_limit, uint8_t *remote_err);


/**
  * @brief  Set limit of absolute position for finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  low_limit: low limit of absolute position
  * @param  high_limit: high limit of absolute position
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPosLimit(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t low_limit, uint16_t high_limit, uint8_t *remote_err);


/**
  * @brief  Start finger in case of stuck.
  * @note   Fingers will auto start after about 500ms when stuck. 
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id_bits: bits of fingers to start
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_FingerStart(void *ctx, uint8_t hand_id, uint8_t finger_id_bits, uint8_t *remote_err);


/**
  * @brief  Stop finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id_bits: bits of fingers to stop
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_FingerStop(void *ctx, uint8_t hand_id, uint8_t finger_id_bits, uint8_t *remote_err);


/**
  * @brief  Set absolute position for finger
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  raw_pos: raw position of finger
  * @param  speed: moving speed, [0, 255], 255 means move full range in 1 second
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPosAbs(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t raw_pos, uint8_t speed, uint8_t *remote_err);


/**
  * @brief  Move finger to position
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger to move
  * @param  pos: target position, [0, 65535], mapped to [low_limit, high_limit] internally
  * @param  speed: moving speed, [0, 255], 255 means move full range in 1s (0.7s for thumb root)
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPos(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t pos, uint8_t speed, uint8_t *remote_err);

/**
  * @brief  Set finger angle
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger to move
  * @param  angle: target angle, value = real angle * 100
  * @param  speed: moving speed, [0, 255], 255 means move full range in 1s (0.7s for thumb root)
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerAngle(void *ctx, uint8_t hand_id, uint8_t finger_id, int16_t angle, uint8_t speed, uint8_t *remote_err);


/**
  * @brief  Set thumb root pos
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  pos: 0, 1 or 2
  * @param  speed: moving speed, [0, 255], 255 means move full range in 0.7s
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetThumbRootPos(void *ctx, uint8_t hand_id, uint8_t pos, uint8_t speed, uint8_t *remote_err);


/**
  * @brief  Set absolute positions for all fingers
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  raw_pos: raw position of finger
  * @param  speed: moving speed, [0, 255], 255 means move full range in 1 second
  * @param  motor_cnt: motor count, i.e., array size of raw_pos[] or speed[]
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPosAbsAll(void *ctx, uint8_t hand_id, uint16_t *raw_pos, uint8_t *speed, uint8_t motor_cnt, uint8_t *remote_err);


/**
  * @brief  Move all fingers to specified position
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  pos: target position, [0, 65535], mapped to [low_limit, high_limit] internally
  * @param  speed: moving speed, [0, 255], 255 means move full range in 1 second
  * @param  motor_cnt: motor count, i.e., array size of pos[] or speed[]
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPosAll(void *ctx, uint8_t hand_id, uint16_t *pos, uint8_t *speed, uint8_t motor_cnt, uint8_t *remote_err);


/**
  * @brief  Set first joint angles for all fingers
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  angle: target angle, value = real angle * 100
  * @param  speed: moving speed, [0, 255], 255 means move full range in 1 second
  * @param  motor_cnt: motor count, i.e., array size of angle[] or speed[]
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerAngleAll(void *ctx, uint8_t hand_id, int16_t *angle, uint8_t *speed, uint8_t motor_cnt, uint8_t *remote_err);


/**
  * @brief  Custom command
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  data: address of data
  * @param  send_data_size: send data size
  * @param  recv_data_size: address of recv data size
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetCustom(void *ctx, uint8_t hand_id, uint8_t* data, uint8_t send_data_size, uint8_t *recv_data_size, uint8_t *remote_err);


/**
  * @brief  Set self-test level
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  self_test_level: self-test level, 0: wait for command HAND_CMD_START_INIT to init; 1: semi self-test on power up; 2: full self-test on power up
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetSelfTestLevel(void *ctx, uint8_t hand_id, uint8_t self_test_level, uint8_t *remote_err);


/**
  * @brief  Set beep ON/OFF
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  beep_on: beep on/off
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetBeepSwitch(void *ctx, uint8_t hand_id, uint8_t beep_on, uint8_t *remote_err);


/**
  * @brief  Beep
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  duration: beep duration in milliseconds
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_Beep(void *ctx, uint8_t hand_id, uint16_t duration, uint8_t *remote_err);


/**
  * @brief  Set button pressed count, usually used when calibrating robotic hand
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  pressed_cnt: button pressed count
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetButtonPressedCnt(void *ctx, uint8_t hand_id, uint8_t pressed_cnt, uint8_t *remote_err);


 /**
  * @brief  Start init in case of SELF_TEST_SWITCH=0
  * @note
  * @param  ctx: pointer to OHand context
  * @param  hand_id: id of OHand
  * @param  remote_err: address to put remote node error in case of  HAND_RESP_HAND_ERROR
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_StartInit(void *ctx, uint8_t hand_id, uint8_t *remote_err);


#ifdef __cplusplus
}
#endif
#endif //__HAND_SERIAL_API_H__

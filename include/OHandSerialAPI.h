
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


/* Node error callback function prototype */
typedef void (*HAND_NODE_ERROR_CALLBACK)(uint8_t node_addr, uint8_t err);


/* Exported constants --------------------------------------------------------*/

typedef enum
{
    HAND_PROTOCOL_UART,
    HAND_PROTOCOL_I2C
}   HAND_PROTOCOL;



/* Only matched major version could work */
#define FW_VER_MAJOR_MIN 2
#define FW_VER_MAJOR_MAX 3


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
#define HAND_RESP_INTERFACE_NOT_SET                     0x02           /* local error, send data function not set, call HAND_SetDataInterface(...) first */
#define HAND_RESP_TIMEOUT                               0x03           /* local error, time out when waiting node response */
#define HAND_RESP_INVALID_OUT_BUFFER_SIZE               0x04           /* local error, out buffer size not matched to returned data */
#define HAND_RESP_UNMATCHED_ADDR                        0x05           /* local error, unmatched node id between returned and waiting */
#define HAND_RESP_UNMATCHED_CMD                         0x06           /* local error, unmatched command between returned and waiting */
#define HAND_RESP_DATA_SIZE_TOO_BIG                     0x07           /* local error, size of data to send exceeds the buffer size */
#define HAND_RESP_DATA_INVALID                          0x08           /* local error, data content invalid */



/* Exported macro ------------------------------------------------------------*/



/* Exported functions ------------------------------------------------------- */

/*
 * OHand Serial API
 */

/**
  * @brief  Set send data function and receive data function
  * @note   For none interrupt receive function, pass receive function pointer as parameter
  *         RecvDataImpl and call HAND_OnData(...) in receive function. For interrupt receive
  *         mode, pass NULL as parameter RecvDataImpl and call HAND_OnData(...) in ISR. Call
  *         this before using other get & set functions
  * @param  protocol: HAND_PROTOCOL_UART or HAND_PROTOCOL_I2C
  * @param  address_master: master node address
  * @param  SendDataImpl: send data function pointer
  * @param  RecvDataImpl: receive data function pointer, NULL for interrupt receive mode
  * @retval None
  */
void HAND_SetDataInterface(
  HAND_PROTOCOL protocol,
  uint8_t address_master,
  void (*SendDataImpl)(uint8_t addr, uint8_t *data, uint8_t size),
  void (*RecvDataImpl)()
);


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
  * @param  timeout: timeout in miliseconds
  * @param  DelayMilliSecondsImpl: pointer of function to delay miliseconds
  * @retval None
  */
void HAND_SetCommandTimeOut(uint16_t timeout);


/**
  * @brief  Feed data to internal parser
  * @note   Call this in data receive function, either RecvDataImpl or ISR
  * @param  data: data byte received
  * @retval None
  */
void HAND_OnData(uint8_t data);



/*
 * Gets
 */

/**
  * @brief  Get firmware version
  * @note
  * @param  hand_id: id of OHand
  * @param  major: pointer to uint8_t type var, when success, the major version of firmware will be stored
  * @param  minor: pointer to uint8_t type var, when success, the minor version of firmware will be stored
  * @param  revision: pointer to uint16_t type var, when success, the revision of firmware will be stored
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFirmwareVersion(uint8_t hand_id, uint8_t *major, uint8_t *minor, uint16_t *revision, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get hardware version
  * @note
  * @param  hand_id: id of OHand
  * @param  hw_type: pointer to uint8_t type var, when success, the hardware type will be stored
  * @param  hw_ver: pointer to uint8_t type var, when success, the hardware version will be stored
  * @param  boot_version: pointer to uint16_t type var, when success, the major version will be stored in high byte,
  *                  while the minor ver in the low byte 
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetHardwareVersion(uint8_t hand_id, uint8_t *hw_type, uint8_t *hw_ver, uint16_t *boot_version, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get calibration data
  * @note
  * @param  hand_id: id of OHand
  * @param  end_pos: address of array to put end_pos
  * @param  start_pos: address of array to put start_pos
  * @param  thumb_root_pos: address of array to put thumb_root_pos
  * @param  motor_cnt: motor count, i.e., array size of end_pos[] or start_pos[]
  * @param  thumb_root_pos_cnt: count of pre-defined thumb root pos
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetCaliData(uint8_t hand_id, uint16_t *end_pos, uint16_t *start_pos, uint16_t *thumb_root_pos, uint16_t motor_cnt, uint16_t thumb_root_pos_cnt, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get PID gain
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  p: address of var to put PID gain p
  * @param  i: address of var to put PID gain i
  * @param  d: address of var to put PID gain d
  * @param  g: address of var to put PID gain g
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPID(uint8_t hand_id, uint8_t finger_id, float *p, float *i, float *d, float *g, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get current limit of finger
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  current_limit: address of var to put current limit in mA
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerCurrentLimit(uint8_t hand_id, uint8_t finger_id, uint16_t *current_limit, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get current of finger
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  current: address of var to put current in mA
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerCurrent(uint8_t hand_id, uint8_t finger_id, uint16_t *current, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get force limit of finger
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  force_limit: address of var to put force limit in mN
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerForceLimit(uint8_t hand_id, uint8_t finger_id, uint16_t *force_limit, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get force of finger
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  force: address of var to put force in mN
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerForce(uint8_t hand_id, uint8_t finger_id, uint16_t *force, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get absolute position limit of finger
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  low_limit: address of var to put low limit of finger position
  * @param  high_limit: address of var to put high limit of finger position
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPosLimit(uint8_t hand_id, uint8_t finger_id, uint16_t *low_limit, uint16_t *high_limit, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get absolute position of finger
  * @note
  * @param  hand_id: id of OHand
  * @param  target_pos: address of var to put raw target position, i.e., encoder value of finger
  * @param  current_pos: address of var to put raw current position, i.e., encoder value of finger
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPosAbs(uint8_t hand_id, uint8_t finger_id, uint16_t *target_pos, uint16_t *current_pos, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get logical position of finger
  * @note
  * @param  hand_id: id of OHand
  * @param  target_pos: address of var to put logical target position
  * @param  current_pos: address of var to put logical current position
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerPos(uint8_t hand_id, uint8_t finger_id, uint16_t *target_pos, uint16_t *current_pos, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get logical position of finger
  * @note
  * @param  hand_id: id of OHand
  * @param  target_angle: address of var to put logical target position, value = real angle * 100
  * @param  current_angle: address of var to put logical current position, value = real angle * 100
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetFingerAngle(uint8_t hand_id, uint8_t finger_id, uint16_t *target_angle, uint16_t *current_angle, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get position of thumb's root
  * @note
  * @param  hand_id: id of OHand
  * @param  raw_encoder: address of var to put raw encoder value of thumb's root, value range [0-65535]
  * @param  pos: address of var to put mapped position of thumb's root, value range [0-2], or 255 if invalid
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetThumbRootPos(uint8_t hand_id, uint16_t *raw_encoder, uint8_t *pos, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get beep switch
  * @note
  * @param  hand_id: id of OHand
  * @param  self_test_on: address of var to put self-test switch status
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetSelfTestSwitch(uint8_t hand_id, uint8_t *self_test_switch, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get beep switch
  * @note
  * @param  hand_id: id of OHand
  * @param  beep_on: address of var to put beep switch status
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetBeepSwitch(uint8_t hand_id, uint8_t *beep_switch, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get button pressed count of OHand
  * @note
  * @param  hand_id: id of OHand
  * @param  pressed_cnt: button pressed count
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetButtonPressedCnt(uint8_t hand_id, uint8_t *pressed_cnt, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get the 96 bits device UID
  * @note
  * @param  hand_id: id of OHand
  * @param  uid_w0: pointer to word 0 of UID
  * @param  uid_w1: pointer to word 1 of UID
  * @param  uid_w2: pointer to word 2 of UID
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetUID(uint8_t hand_id, uint32_t *uid_w0, uint32_t *uid_w1, uint32_t *uid_w2, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get battery voltage
  * @note
  * @param  hand_id: id of OHand
  * @param  voltage: pointer to uint16_t type var, when success, the voltage in mv will be stored
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetBatteryVoltage(uint8_t hand_id, uint16_t *voltage, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get usage stat
  * @note
  * @param  hand_id: id of OHand
  * @param  total_use_time: address of var to put total use time in seconds
  * @param  total_open_times: address of array to put total_open_times[5]
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_GetUsageStat(uint8_t hand_id, uint32_t *total_use_time, uint32_t *total_open_times, HAND_NODE_ERROR_CALLBACK error_callback);



/*
 * Sets
 */

 /**
  * @brief  Reset hand
  * @note   NO response, so return code must be timeout
  * @param  hand_id: id of OHand
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_Reset(uint8_t hand_id, HAND_NODE_ERROR_CALLBACK error_callback);


 /**
  * @brief  Turn hand off
  * @note   NO response, so return code must be timeout
  * @param  hand_id: id of OHand
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_PowerOff(uint8_t hand_id, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set hand id
  * @note   Need to reboot device after setting
  * @param  hand_id: id of OHand
  * @param  new_id: new id for hand
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetID(uint8_t hand_id, uint8_t new_id, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Start calibration
  * @note
  * @param  hand_id: id of OHand
  * @param  key: key to enable calibration, to avoid mishandling
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_Calibrate(uint8_t hand_id, uint16_t key, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set calibration data
  * @note
  * @param  hand_id: id of OHand
  * @param  end_pos: address of array to put end_pos
  * @param  start_pos: address of array to put start_pos
  * @param  thumb_root_pos: address of array to put thumb_root_pos
  * @param  motor_cnt: motor count, i.e., array size of end_pos[] or start_pos[]
  * @param  thumb_root_pos_cnt: count of pre-defined thumb root pos
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetCaliData(uint8_t hand_id, uint16_t *end_pos, uint16_t *start_pos, uint16_t *thumb_root_pos, uint16_t motor_cnt, uint16_t thumb_root_pos_cnt, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set PID gain
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  p: PID gain p
  * @param  i: PID gain i
  * @param  d: PID gain d
  * @param  d: PID gain g
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPID(uint8_t hand_id, uint8_t finger_id, float p, float i, float d, float g, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set finger for limit
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  current_limit: current limit in mA for finger
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerCurrentLimit(uint8_t hand_id, uint8_t finger_id, uint16_t current_limit, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set finger for limit
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger
  * @param  force_limit: force limit in mN for finger
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerForceLimit(uint8_t hand_id, uint8_t finger_id, uint16_t force_limit, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set limit of absolute position for finger
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  low_limit: low limit of absolute position
  * @param  high_limit: high limit of absolute position
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPosLimit(uint8_t hand_id, uint8_t finger_id, uint16_t low_limit, uint16_t high_limit, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Start finger in case of stuck.
  * @note   Fingers will auto start after about 500ms when stuck. 
  * @param  hand_id: id of OHand
  * @param  finger_id_bits: bits of fingers to start
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_FingerStart(uint8_t hand_id, uint8_t finger_id_bits, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Stop finger
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id_bits: bits of fingers to stop
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_FingerStop(uint8_t hand_id, uint8_t finger_id_bits, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set absolute position for finger
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: id of finger
  * @param  raw_pos: raw position of finger
  * @param  speed: moving speed, [0, 255], 255 means move full range in 1 second
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPosAbs(uint8_t hand_id, uint8_t finger_id, uint16_t raw_pos, uint8_t speed, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Move finger to position
  * @note
  * @param  hand_id: id of OHand
  * @param  finger_id: index of finger to move
  * @param  pos: target position, [0, 65535], mapped to [low_limit, high_limit] internally
  * @param  speed: moving speed, [0, 255], 255 means move full range in 1 second
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetFingerPos(uint8_t hand_id, uint8_t finger_id, uint16_t pos, uint8_t speed, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set self-test ON/OFF
  * @note
  * @param  hand_id: id of OHand
  * @param  beep_on: self-test on/off
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetSelfTestSwitch(uint8_t hand_id, uint8_t self_test_on, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set beep ON/OFF
  * @note
  * @param  hand_id: id of OHand
  * @param  beep_on: beep on/off
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetBeepSwitch(uint8_t hand_id, uint8_t beep_on, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Beep
  * @note
  * @param  hand_id: id of OHand
  * @param  duration: beep duration in milliseconds
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_Beep(uint8_t hand_id, uint16_t duration, HAND_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set button pressed count, usually used when calibrating robotic hand
  * @note
  * @param  hand_id: id of OHand
  * @param  pressed_cnt: button pressed count
  * @param  error_callback: callback function of type HAND_NODE_ERROR_CALLBACK, can be NULL
  * @retval -HAND_RESP_SUCCESS: success
  *         -Others: failed, see definition of API return values
  */
uint8_t HAND_SetButtonPressedCnt(uint8_t hand_id, uint8_t pressed_cnt, HAND_NODE_ERROR_CALLBACK error_callback);


#ifdef __cplusplus
}
#endif
#endif //__HAND_SERIAL_API_H__

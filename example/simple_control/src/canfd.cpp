#include "canfd.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include <ControlCANFD.h>
#include "OHandSerialAPI.h"

#include <stdio.h>
#include <string.h>

// use dbit baud rate
#ifndef CANFD_BRS
#define CANFD_BRS 0x01
#endif


#define CANFD_DEVICE_TYPE ZCAN_USBCANFD_200U

#define CANFD_MAX_CHANNEL (2)

#ifndef CANFD_DBIT_BAUD
#define CANFD_DBIT_BAUD (5000000u)
#endif

static const uint8_t kCanfdPayloadLens[] = {12, 16, 20, 24, 32, 48, 64};


typedef struct
{
    DEVICE_HANDLE device;
    CHANNEL_HANDLE channel;
    uint32_t can_index;
} CANFD_Port;

static CANFD_Port canfd_port = {INVALID_DEVICE_HANDLE, INVALID_CHANNEL_HANDLE, 0};

static int CANFD_SetBaudRateByProperty(DEVICE_HANDLE device, uint32_t can_index, uint32_t abit, uint32_t dbit)
{
    IProperty* prop;
    char path_abit[64];
    char path_dbit[64];
    char value[32];
    const char* read_back;
    int ok;

    prop = GetIProperty(device);
    if (prop == NULL)
    {
        return 0;
    }

    snprintf(path_abit, sizeof(path_abit), "%u/canfd_abit_baud_rate", can_index);
    snprintf(path_dbit, sizeof(path_dbit), "%u/canfd_dbit_baud_rate", can_index);

    snprintf(value, sizeof(value), "%u", abit);
    ok = (STATUS_OK == prop->SetValue(path_abit, value));

    snprintf(value, sizeof(value), "%u", dbit);
    ok = (STATUS_OK == prop->SetValue(path_dbit, value)) && ok;
    
    read_back = prop->GetValue(path_abit);
    printf("CANFD %s = %s\n", path_abit, (read_back != NULL) ? read_back : "(null)");
    
    read_back = prop->GetValue(path_dbit);
    printf("CANFD %s = %s\n", path_dbit, (read_back != NULL) ? read_back : "(null)");
    
    ReleaseIProperty(prop);
    
    return ok;
}

// Port name format: "<device_index>" or "<device_index>:<channel>", e.g. "0", "0:1".
static int CANFD_ParsePortName(const char* port_name, uint32_t* device_index, uint32_t* can_index)
{
    char extra = 0;
    int matched;

    *device_index = 0;
    *can_index = 0;

    if (port_name == NULL)
    {
        return 0;
    }

    matched = sscanf(port_name, "%u:%u%c", device_index, can_index, &extra);

    if (matched == 1)
    {
        // Only device index given, use channel 0 by default.
        *can_index = 0;
        return 1;
    }

    return matched == 2;
}

static uint32_t CANFD_CalEffectiveLength(uint32_t size)
{
    if (size <= 8) return size;

    for (int i = 0; i < sizeof(kCanfdPayloadLens) / sizeof(kCanfdPayloadLens[0]); i++)
    {
        if (size <= kCanfdPayloadLens[i])
        {
            return kCanfdPayloadLens[i];
        }
    }
    
    return CANFD_MAX_DLEN;
}

// Initialize
void* CANFD_Init(const char* port_name, uint32_t baudrate)
{
    uint32_t device_index = 0;
    uint32_t can_index = 0;
    ZCAN_CHANNEL_INIT_CONFIG cfg;

    if (canfd_port.channel != INVALID_CHANNEL_HANDLE)
    {
        return &canfd_port;
    }

    if (!CANFD_ParsePortName(port_name, &device_index, &can_index))
    {
        printf("Error: invalid port '%s'. Expected '<device_index>' or '<device_index>:<channel>', e.g. 0 or 0:1.\n",
               (port_name != NULL) ? port_name : "");
        return NULL;
    }

    if (can_index >= CANFD_MAX_CHANNEL)
    {
        printf("Error: channel index must be 0 or 1.\n");
        return NULL;
    }

    canfd_port.device = ZCAN_OpenDevice(CANFD_DEVICE_TYPE, device_index, 0);
    if (canfd_port.device == INVALID_DEVICE_HANDLE)
    {
        printf("Error: ZCAN_OpenDevice failed. Check the USB driver and device index %u.\n", device_index);
        return NULL;
    }
    
    if (!CANFD_SetBaudRateByProperty(canfd_port.device, can_index, baudrate, CANFD_DBIT_BAUD))
    {
        printf("Warning: IProperty baud rate failed, fallback to ZCAN_SetAbitBaud/ZCAN_SetDbitBaud.\n");

        if (STATUS_OK != ZCAN_SetAbitBaud(canfd_port.device, can_index, baudrate)
          || STATUS_OK != ZCAN_SetDbitBaud(canfd_port.device, can_index, CANFD_DBIT_BAUD))
        {
            printf("Error: failed to set CAN FD baud rate, abit %u, dbit %u.\n", baudrate, CANFD_DBIT_BAUD);
            CANFD_DeInit();
            return NULL;
        }
    }

    // 0: ISO CAN FD, 1: Bosch CAN FD
    ZCAN_SetCANFDStandard(canfd_port.device, can_index, 0);

    memset(&cfg, 0, sizeof(cfg));
    cfg.can_type = TYPE_CANFD; // FD device
    cfg.canfd.mode = 0; // Normal mode
    cfg.canfd.filter = 0;
    cfg.canfd.acc_code = 0;
    cfg.canfd.acc_mask = 0xFFFFFFFFu; // Accept all frames

    canfd_port.channel = ZCAN_InitCAN(canfd_port.device, can_index, &cfg);
    if (canfd_port.channel == INVALID_CHANNEL_HANDLE)
    {
        printf("Error: ZCAN_InitCAN failed.\n");
        CANFD_DeInit();
        return NULL;
    }

    if (STATUS_ERR == ZCAN_StartCAN(canfd_port.channel))
    {
        printf("Error: ZCAN_StartCAN failed.\n");
        CANFD_DeInit();
        return NULL;
    }
    
    ZCAN_ClearBuffer(canfd_port.channel);

    canfd_port.can_index = can_index;

    printf("CANFD device %u channel %u opened, abit %u bps, dbit %u bps.\n",
           device_index, can_index, baudrate, CANFD_DBIT_BAUD);

    return &canfd_port;
}

// Release resources
void CANFD_DeInit()
{
    if (canfd_port.channel != INVALID_CHANNEL_HANDLE)
    {
        ZCAN_ResetCAN(canfd_port.channel);
        canfd_port.channel = INVALID_CHANNEL_HANDLE;
    }

    if (canfd_port.device != INVALID_DEVICE_HANDLE)
    {
        ZCAN_CloseDevice(canfd_port.device);
        canfd_port.device = INVALID_DEVICE_HANDLE;
    }

    canfd_port.can_index = 0;
}

// Split-frame transmission, maximum 64 bytes payload per CAN FD frame
void CANFD_SendData(uint8_t addr, uint8_t* data, uint8_t len, void* hand_ctx)
{
    ZCAN_TransmitFD_Data canfdMessage;
    uint8_t payload;

    if (canfd_port.channel == INVALID_CHANNEL_HANDLE || data == NULL || len == 0)
    {
        return;
    }

    if (len > CANFD_MAX_DLEN)
    {
        printf("Error: packet too long for one CAN FD frame, len=%d.\n", len);
        return;
    }

    payload = (uint8_t)CANFD_CalEffectiveLength(len);

    memset(&canfdMessage, 0, sizeof(canfdMessage));
    canfdMessage.frame.can_id = MAKE_CAN_ID(addr, 0, 0, 0); // standard frame, data frame
    canfdMessage.frame.len = payload;
    canfdMessage.frame.flags = CANFD_BRS; // data phase at dbit baud rate, must match the tool
    memcpy(canfdMessage.frame.data, data, len); // the rest of the payload stays 0x00
    canfdMessage.transmit_type = 0; // normal transmission

    if (ZCAN_TransmitFD(canfd_port.channel, &canfdMessage, 1) != 1)
    {
        printf("ZCAN_TransmitFD failed.\n");
        return;
    }

    printf("Send frame: ID=0x%03X, LEN=%d (payload %d), FLAGS=0x%02X, DATA=",
           (unsigned int)GET_ID(canfdMessage.frame.can_id), len, payload, canfdMessage.frame.flags);

    for (uint8_t i = 0; i < canfdMessage.frame.len; i++)
    {
        printf("%02X ", canfdMessage.frame.data[i]);
    }

    printf("\n");
}

// Receive frame data
void CANFD_RecvData(void* hand_ctx)
{
    if (canfd_port.channel == INVALID_CHANNEL_HANDLE)
    {
        return;
    }

    // Drain the receive queue without blocking, wait_time 0 means no wait.
    while (ZCAN_GetReceiveNum(canfd_port.channel, TYPE_CANFD) > 0)
    {
        ZCAN_ReceiveFD_Data message;
        uint8_t i;

        memset(&message, 0, sizeof(message));

        if (ZCAN_ReceiveFD(canfd_port.channel, &message, 1, 0) != 1)
        {
            return;
        }

        // Error frames carry bus diagnostics (ACK error, bus off, stuff error...).
        // Print them instead of dropping silently.
        if (IS_ERR(message.frame.can_id))
        {
            printf("Error frame: code=0x%08X, len=%d\n",
                   (unsigned int)GET_ID(message.frame.can_id), message.frame.len);
            continue;
        }

        // Ignore remote frames.
        if (IS_RTR(message.frame.can_id))
        {
            continue;
        }
        // ... existing code ...
        for (i = 0; i < message.frame.len && i < CANFD_MAX_DLEN; i++)
        {
            HAND_OnData(hand_ctx, message.frame.data[i]);
        }
    }

    // Some driver versions put classic CAN frames (len <= 8) into the CAN queue
    // even when the channel was initialized as TYPE_CANFD.
    while (ZCAN_GetReceiveNum(canfd_port.channel, TYPE_CAN) > 0)
    {
        ZCAN_Receive_Data message;
        uint8_t i;

        memset(&message, 0, sizeof(message));

        if (ZCAN_Receive(canfd_port.channel, &message, 1, 0) != 1)
        {
            return;
        }

        if (IS_ERR(message.frame.can_id))
        {
            printf("Error frame (CAN): code=0x%08X, len=%d\n",
                   (unsigned int)GET_ID(message.frame.can_id), message.frame.can_dlc);
            continue;
        }

        if (IS_RTR(message.frame.can_id))
        {
            continue;
        }

        printf("Receive classic frame: ID=0x%03X, LEN=%d, DATA=",
               (unsigned int)GET_ID(message.frame.can_id), message.frame.can_dlc);

        for (i = 0; i < message.frame.can_dlc && i < CAN_MAX_DLEN; i++)
        {
            printf("%02X ", message.frame.data[i]);
        }

        printf("\n");

        for (i = 0; i < message.frame.can_dlc && i < CAN_MAX_DLEN; i++)
        {
            HAND_OnData(hand_ctx, message.frame.data[i]);
        }
    }
}

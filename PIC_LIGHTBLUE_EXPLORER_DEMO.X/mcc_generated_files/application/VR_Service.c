#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "VR_Service.h"
#include "LIGHTBLUE_service.h"
// APP Dependencies
#include "BMA253_accel.h"
#include "MCP9844_temp_sensor.h"
#include "pin_manager.h"
#include "drivers/uart.h"
#include "../mcc_generated_files/eusart1.h"

#define Hex(x) _hex[(x) & 0xF]
#define Ascii2Decimal(c)  (((c) <= '9') ? (c) - '0' : (c & 0x5f) - 'A' + 10)

#define DataLedOn()                 DATA_LED_SetLow()
#define DataLedOff()                DATA_LED_SetHigh()

#define GpioInput_1()               GPIO_1_GetValue()
#define GpioInput_2()               GPIO_2_GetValue()

#define START_BYTE                  '{'
#define TERMINATION_BYTE            '}'

#define LED_OFF_STATE               (0x01)
#define NOT_PRESSED_STATE           (0x01)

#define VR_OFF                      (0x00)

#define DATA_LED_IDENTIFIER         (0x00)
#define ERROR_LED_IDENTIFIER        (0x10)

#define NIBBLE_MASK                 (0x01)

const char * const vr_protocol_version_number      = "0.0.1";

typedef enum
{
    GPIO_STATE_ID           = 'G',
    LED_STATE_ID            = 'L',
    BUTTON_STATE_ID         = 'P',
    TEMPERATURE_DATA_ID     = 'T',
    A_DATA_ID               = 'A',
    X_DATA_ID               = 'X',
    Y_DATA_ID               = 'Y',
    Z_DATA_ID               = 'Z',
    SERIAL_DATA_ID          = 'S',
    ERROR_ID                = 'R',
    PROTOCOL_VERSION_ID     = 'V',
    UI_CONFIG_DATA_ID       = 'U'
}PROTOCOL_PACKET_TYPES_t;

typedef enum
{
    IDLE                    = 0,
    SEQUENCE_NUMBER         = 1,
    PACKET_ID               = 2,
    PAYLOAD_SIZE_0          = 3,
    PAYLOAD_SIZE_1          = 4,
    PAYLOAD_0               = 5,
    PAYLOAD_1               = 6        
           
}PACKET_PARSER_STATE_t;

static char _hex[] = "0123456789ABCDEF";
static uint8_t sequenceNumber = 0;

static void VR_SendPacket(char packetID, char* payload);
static void VR_SplitWord(char* payload, int16_t value);
static void VR_SplitByte(char* payload, int8_t value);
static uint8_t VR_GetButtonValue(void);
static uint8_t VR_GetDataLedValue(void);
static uint8_t VR_GetErrorLedValue(void);
static void VR_SetErrorLedValue(bool value);
static void VR_PerformAction(char id, uint8_t data);

static void VR_SendData(char packetID, char* payload);
static void VR_AppendData(void);

const iVR_FunctionPtrs_t VR_BRIDGE = {
    .bridgeTx = EUSART1_Write,
    .bridgeRx = EUSART1_Read,
};

void VR_AllData(bool connectedState)
{
    uint8_t button;
    char payloadButton[3];
    char payloadLed[3];
    uint8_t led;
    char payloadGpio[3];
    uint8_t gpio;
    char payloadTemp[5];
    int16_t temperature;
    char payloadAccel[13];
    BMA253_ACCEL_DATA_t accelData;
    
    uint8_t fixError;
    uint8_t dataLed;
    
    button = VR_GetButtonValue();
    *payloadButton = '\0';
    
    fixError = (VR_GetErrorLedValue() & 0x01);
    if (fixError == 0x01)
    {
        fixError = 0x00;
    }
    else
    {
        fixError = 0x01;
    }
    
    dataLed = VR_GetDataLedValue();
    led = (0x07 & ( ((connectedState & 0x01) << 2) + ((fixError & 0x01) << 1) + (dataLed & 0x01) ) );
    *payloadLed = '\0';
    
    gpio = (0x03 & ((GpioInput_2() << 1) + GpioInput_1()));
    *payloadGpio = '\0';
    
    MCP9844_GetTemperatureValue(&temperature);
    *payloadTemp = '\0';    
    
    *payloadAccel = '\0';
    BMA253_GetAccelDataXYZ(&accelData);
    
    VR_BRIDGE.bridgeTx(START_BYTE);
    VR_SplitByte(payloadButton, button);
    VR_SendData(BUTTON_STATE_ID, payloadButton);
    VR_AppendData();
    VR_SplitByte(payloadLed, led);  
    VR_SendData(LED_STATE_ID, payloadLed);
    VR_AppendData();
    VR_SplitByte(payloadGpio, gpio);  
    VR_SendData(GPIO_STATE_ID, payloadGpio);
    VR_AppendData();
    VR_SplitWord(payloadTemp, temperature);
    VR_SendData(TEMPERATURE_DATA_ID, payloadTemp);
    VR_AppendData();
    VR_SplitWord(payloadAccel, (accelData.x & 0x0FFF)); // Masking to ensure top nibble is always 0 as light blue expects
    VR_SplitWord(payloadAccel, (accelData.y & 0x0FFF)); // TODO: JIRA M8TS-2071 Light Blue seems to be throwing an exception when it is not 0
    VR_SplitWord(payloadAccel, (accelData.z & 0x0FFF));
    VR_SendData(A_DATA_ID, payloadAccel);
    VR_BRIDGE.bridgeTx(TERMINATION_BYTE);
    VR_BRIDGE.bridgeTx('\r');
    VR_BRIDGE.bridgeTx('\n');
}
void VR_TemperatureSensor(void)
{
    char payload[5];
    int16_t temperature;
    
    *payload = '\0';
    MCP9844_GetTemperatureValue(&temperature);
    VR_SplitWord(payload, temperature);
    VR_SendPacket(TEMPERATURE_DATA_ID, payload);
}

void VR_AccelSensor(void)
{
    char payload[13];
    BMA253_ACCEL_DATA_t accelData;
    
    *payload = '\0';
    BMA253_GetAccelDataXYZ(&accelData);
    
    VR_SplitWord(payload, (accelData.x & 0x0FFF)); // Masking to ensure top nibble is always 0 as light blue expects
    VR_SplitWord(payload, (accelData.y & 0x0FFF)); // TODO: JIRA M8TS-2071 Light Blue seems to be throwing an exception when it is not 0
    VR_SplitWord(payload, (accelData.z & 0x0FFF));
    VR_SendPacket(A_DATA_ID, payload);
}

void VR_PushButton(void)
{
    char payload[3];
    uint8_t button = VR_GetButtonValue();
    
    *payload = '\0';
    VR_SplitByte(payload, button);
    VR_SendPacket(BUTTON_STATE_ID, payload);
}

void VR_LedState(void)
{
    char payload[3];
    uint8_t led;
    uint8_t ledDATA;
    uint8_t ledERR;
    
    ledDATA = VR_GetDataLedValue();
    ledERR = VR_GetErrorLedValue();
    if (ledERR == 0x01)
    {
        ledERR = 0;
    }
    else
    {
        ledERR = 1;
    }
    led = (0x03 & ((ledERR << 1) + (ledDATA)));
    *payload = '\0';
    VR_SplitByte(payload, led);  
    VR_SendPacket(LED_STATE_ID, payload);
}

void VR_GpioState(void)
{
    char payload[3];
    uint8_t gpio;

    gpio = (0x03 & ((GpioInput_2() << 1) + GpioInput_1()));
    *payload = '\0';
    VR_SplitByte(payload, gpio);  
    VR_SendPacket(GPIO_STATE_ID, payload);
}

void VR_SendSerialData(char* serialData)
{
    uint8_t length = strlen(serialData) * 2;
    
    VR_BRIDGE.bridgeTx(START_BYTE);
    VR_BRIDGE.bridgeTx(SERIAL_DATA_ID);
    VR_BRIDGE.bridgeTx(',');
    while(*serialData)
    {
        VR_BRIDGE.bridgeTx(Hex(*serialData >> 4));
        VR_BRIDGE.bridgeTx(Hex(*serialData++));
    }
    VR_BRIDGE.bridgeTx(TERMINATION_BYTE);
    VR_BRIDGE.bridgeTx('\r');
    VR_BRIDGE.bridgeTx('\n');
}
static char serialArray[32] = {0x00};
static uint8_t serialIndex = 0;
static bool VR_CheckMessage(void);

void VR_OpenSerial(void)
{
    VR_BRIDGE.bridgeTx(START_BYTE);
    VR_BRIDGE.bridgeTx(SERIAL_DATA_ID);
    VR_BRIDGE.bridgeTx(':');
    serialIndex = 0;
}

void VR_SendCharacter(uint8_t data)
{
    serialArray[serialIndex++] = data;
    VR_BRIDGE.bridgeTx(data);
}

void VR_CloseSerial(void)
{
    VR_BRIDGE.bridgeTx(TERMINATION_BYTE);
    VR_BRIDGE.bridgeTx('\r');
    VR_BRIDGE.bridgeTx('\n');
    serialArray[serialIndex] = '\0';
    VR_CheckMessage();
}

bool VR_SerialData(char* serialData)
{
    if (strcmp(serialData, "led data off") == 0)
    {
        DataLedOff();
        return true;
    }
    else if (strcmp(serialData, "led data on") == 0)
    {
        DataLedOn();
        return true;
    }
    return false;
}

static bool VR_CheckMessage(void)
{
    if (strcmp(serialArray, "Hello") == 0)
    {
        VR_BRIDGE.bridgeTx('h');
        VR_BRIDGE.bridgeTx('i');
        return true;
    }
    else if (strcmp(serialArray, "led data off") == 0)
    {
        DataLedOff();
        return true;
    }
    else if (strcmp(serialArray, "led data on") == 0)
    {
        DataLedOn();
        return true;
    }
    else if (strcmp(serialArray, "led err off") == 0)
    {
        LIGHTBLUE_SetErrorLedValue(false);
        return true;
    }
    else if (strcmp(serialArray, "led err on") == 0)
    {
        LIGHTBLUE_SetErrorLedValue(true);
        return true;
    }
    return false;
}

void VR_ParseIncomingPacket(char receivedByte)
{
    static PACKET_PARSER_STATE_t parserState = IDLE;
    static uint8_t length = 0;
    static uint16_t data = 0;
    static char packetID = '\0';

    switch(parserState) 
    {
        case SEQUENCE_NUMBER:
            //ignore sequence
            parserState = PACKET_ID;
            break;
        case PACKET_ID:
            packetID = receivedByte;
            parserState = PAYLOAD_SIZE_0;
            break;
        case PAYLOAD_SIZE_0:
            length = Ascii2Decimal(receivedByte);
            parserState = PAYLOAD_SIZE_1;
            break;
        case PAYLOAD_SIZE_1:
            length = (length << 4) + Ascii2Decimal(receivedByte);
            parserState = PAYLOAD_0;
            break;
        case PAYLOAD_0:
            data = Ascii2Decimal(receivedByte);
            length--;
            if (length == 0)
            {
                parserState = IDLE;
            }
            else
            {
                parserState = PAYLOAD_1;
            }
            break;
        case PAYLOAD_1:
            data = (data << 4) + Ascii2Decimal(receivedByte);
            VR_PerformAction(packetID, data);
            length--;
            if (length == 0)
            {
                parserState = IDLE;
            }
            else
            {
                parserState = PAYLOAD_0;
            }
            break;
        case IDLE:
        default:
            if (receivedByte == START_BYTE)
            {
                parserState = SEQUENCE_NUMBER;
            }
            break;
    }
}

static void VR_SendPacket(char packetID, char* payload)
{
    VR_BRIDGE.bridgeTx(START_BYTE);
    VR_BRIDGE.bridgeTx(packetID);
    VR_BRIDGE.bridgeTx(',');
    while (*payload)
    {
        VR_BRIDGE.bridgeTx((*(uint8_t *)payload++));
    }
    VR_BRIDGE.bridgeTx(TERMINATION_BYTE);
    VR_BRIDGE.bridgeTx('\r');
    VR_BRIDGE.bridgeTx('\n');
}

static void VR_SendData(char packetID, char* payload)
{
    VR_BRIDGE.bridgeTx(packetID);
    VR_BRIDGE.bridgeTx(':');
    while (*payload)
    {
        VR_BRIDGE.bridgeTx((*(uint8_t *)payload++));
    }
}

static void VR_AppendData(void)
{
    VR_BRIDGE.bridgeTx(',');
}

static void VR_SplitWord(char* payload, int16_t value)
{
    VR_SplitByte(payload, value);
    VR_SplitByte(payload, value >> 8);
}

static void VR_SplitByte(char* payload, int8_t value)
{
    payload += strlen(payload);
    *payload++ = Hex(value >> 4);
    *payload++ = Hex(value);
    *payload = '\0';
}

static uint8_t VR_GetButtonValue(void)
{
    return NOT_PRESSED_STATE - SW0_GetValue(); // This is forcing proper data for LightBlue
}

static uint8_t VR_GetDataLedValue(void)
{
    return LED_OFF_STATE - DATA_LED_GetValue(); // This is forcing proper data for LightBlue
}

static uint8_t VR_GetErrorLedValue(void)
{
    return LED_OFF_STATE - LIGHTBLUE_GetErrorLedValue();
}

static void VR_SetErrorLedValue(bool value)
{
    bool ledValue = false;
    ledValue = LED_OFF_STATE - value;
}

static void VR_PerformAction(char id, uint8_t data)
{
    uint8_t led;
    
    switch(id)
    {
        case LED_STATE_ID:
            led = (data >> 4) & NIBBLE_MASK;
            if(led == DATA_LED_IDENTIFIER)
            {
                if((data & NIBBLE_MASK) == VR_OFF)
                {
                    DataLedOff();
                }
                else
                {
                    DataLedOn();
                }
            }
            else
            {
                VR_SetErrorLedValue(data & NIBBLE_MASK);
            }
            break;
        case SERIAL_DATA_ID:
            VR_BRIDGE.bridgeTx(data); // echo out the terminal for now
            break;
        default:
            break;
    }
}

#ifndef VR_SERVICE_H
#define	VR_SERVICE_H

typedef struct
{
    void (*bridgeTx)(uint8_t);
    uint8_t (*bridgeRx)(void);
}iVR_FunctionPtrs_t;

extern const iVR_FunctionPtrs_t VR_BRIDGE;

void VR_TemperatureSensor(void);
void VR_AccelSensor(void);
void VR_PushButton(void);
void VR_LedState(void);
void VR_GpioState(void);
void VR_GpioState(void);
void VR_SendSerialData(char* serialData);
void VR_ParseIncomingPacket(char receivedByte);
void VR_AllData(bool connectedState);
void VR_OpenSerial(void);
void VR_CloseSerial(void);
void VR_SendCharacter(uint8_t data);
bool VR_SerialData(char* serialData);

#endif	/* VR_SERVICE_H */


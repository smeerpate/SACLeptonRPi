#include <stdio.h>
#include <unistd.h>

#include <LEPTON_SDK.h>
#include <LEPTON_SYS.h>
#include <LEPTON_OEM.h>
#include <LEPTON_Types.h>

bool biIsConnected = false;
LEP_CAMERA_PORT_DESC_T i2cPort;


int main()
{
    LEP_RESULT sResult;
    LEP_SYS_FLIR_SERIAL_NUMBER_T sSN;
    LEP_SYS_UPTIME_NUMBER_T sUpTime;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    printf("sResult OpenPort: %d\n", sResult);
    sResult = LEP_RunSysPing(&i2cPort);
    printf("sResult Ping: %d\n", sResult);

    sResult = LEP_GetSysFlirSerialNumber(&i2cPort, &sSN);
    printf("SerialNr: %d\n", sSN);

    sResult = LEP_GetSysCameraUpTime(&i2cPort, &sUpTime);
    printf("Uptime: %d\n", sUpTime);

    sResult = LEP_ClosePort(&i2cPort);
    printf("sResult ClosePort: %d\n", sResult);
}

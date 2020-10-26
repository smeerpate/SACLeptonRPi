#include <stdio.h>
#include <unistd.h>

#include <LEPTON_SDK.h>
#include <LEPTON_SYS.h>
#include <LEPTON_OEM.h>
#include <LEPTON_RAD.h>
#include <LEPTON_Types.h>

bool biIsConnected = false;
LEP_CAMERA_PORT_DESC_T i2cPort;


int main()
{
    LEP_RESULT sResult;
    LEP_SYS_FLIR_SERIAL_NUMBER_T sSN;
    LEP_SYS_UPTIME_NUMBER_T sUpTime;
    LEP_SYS_FPA_TEMPERATURE_CELCIUS_T sFPATempDegC;
    LEP_RAD_TS_MODE_E eShutterMode;
    LEP_RAD_ENABLE_E eRadEnableState;
    LEP_RAD_ENABLE_E eTLinEnaState;
    LEP_RAD_ROI_T sROI;
    LEP_RAD_SPOTMETER_OBJ_KELVIN_T sSpotMeter;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    printf("sResult OpenPort: %d\n", sResult);
    sResult = LEP_RunSysPing(&i2cPort);
    printf("sResult Ping: %d\n", sResult);

    sResult = LEP_GetSysFlirSerialNumber(&i2cPort, &sSN);
    printf("SerialNr: %d\n", (u_int64_t)sSN);

    sResult = LEP_GetSysCameraUpTime(&i2cPort, &sUpTime);
    printf("Uptime: %fs\n", (float)sUpTime/1000.0);

    sResult = LEP_GetSysFpaTemperatureCelcius(&i2cPort, &sFPATempDegC);
    printf("FPA Temperature: %fdegC\n", (float)sFPATempDegC);

    sResult = LEP_GetSysAuxTemperatureCelcius(&i2cPort, &sFPATempDegC);
    printf("Aux Temperature: %fdegC\n", (float)sFPATempDegC);

    sResult = LEP_GetRadTShutterMode(&i2cPort, &eShutterMode);
    printf("Shutter Mode: %i\n", (int)eShutterMode);
    printf("sResult Get Shutter mode: %d\n", sResult);

    sResult = LEP_GetRadEnableState(&i2cPort, &eRadEnableState);
    printf("Rad enable state: %i\n", (int)eRadEnableState);
    printf("sResult Get RAD enable state: %d\n", sResult);

    sResult = LEP_GetRadSpotmeterRoi(&i2cPort, &sROI);
    printf("Start row: %i, start col: %i, end row: %i, end col: %i.\n", sROI.startRow, sROI.startCol, sROI.endRow, sROI.endCol);
    printf("sResult Get RAD ROI: %d\n", sResult);


    sResult = LEP_GetRadTLinearEnableState(&i2cPort, &eTLinEnaState);
    printf("TLinear enable state: %i\n", (int)eTLinEnaState);
    printf("sResult LEP_GetRadTLinearEnableState: %d\n", sResult);

    sResult = LEP_RunRadFFC(&i2cPort);
    printf("sResult Run RAD FFC: %d\n", sResult);

    sResult = LEP_GetRadSpotmeterObjInKelvinX100(&i2cPort, &sSpotMeter);
    printf("Spotmeter Value: %i, Max: %i, Min: %i, Population: %i.\n", sSpotMeter.radSpotmeterValue, sSpotMeter.radSpotmeterMaxValue, sSpotMeter.radSpotmeterMinValue, sSpotMeter.radSpotmeterPopulation);
    printf("sResult Get Spotmeter obj: %d\n", sResult);


//    sResult = LEP_RunSysFFCNormalization(&i2cPort);
//    printf("sResult Run FFC: %d\n", sResult);

    sResult = LEP_ClosePort(&i2cPort);
    printf("sResult ClosePort: %d\n", sResult);
}

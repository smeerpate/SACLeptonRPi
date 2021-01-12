#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <stdio.h>
#include <unistd.h>

#include <LEPTON_SDK.h>
#include <LEPTON_SYS.h>
#include <LEPTON_OEM.h>
#include <LEPTON_Types.h>
#include <LEPTON_RAD.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SPI_PACKET_SIZE 164
#define SPI_PACKET_SIZE_UINT16 (SPI_PACKET_SIZE/2)
#define SPI_PACKETS_PER_FRAME 60
#define SPI_FRAME_SIZE_UINT16 (SPI_PACKET_SIZE_UINT16 * SPI_PACKETS_PER_FRAME)

#define LEPTON_WIDTH 80
#define LEPTON_HEIGHT 60

// Value clamping macro
#define CLAMP(x, low, high) ({\
  __typeof__(x) __x = (x); \
  __typeof__(low) __low = (low);\
  __typeof__(high) __high = (high);\
  __x > __high ? __high : (__x < __low ? __low : __x);\
  })

PyObject* LeptonCCIError;
LEP_CAMERA_PORT_DESC_T i2cPort;
char sError[128];

int miSpiFd = 0;
uint8_t result[SPI_PACKET_SIZE*SPI_PACKETS_PER_FRAME];
uint16_t *frameBuffer;
long mlFbMaxValue;
long mlFbMinValue;

static int SpiOpenPort (char *sPort);
static int SpiClosePort();


// Functions
///////////////////////////////////////////////////
// LeptonCCI_RunRadFfc is a Function without
// arguments. Triggers a radiometry FFC.
///////////////////////////////////////////////////
static PyObject* LeptonCCI_RunRadFfc(PyObject* self) {
    LEP_RESULT sResult;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_RunRadFfc: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_RunRadFFC(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_RunRadFfc: Unable to run RAD FFC. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_RunRadFfc: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    Py_RETURN_NONE;
}

///////////////////////////////////////////////////
// LeptonCCI_RunSysFFCNormalization is a Function without
// arguments. Triggers an FFC Normalization.
///////////////////////////////////////////////////
static PyObject* LeptonCCI_RunSysFFCNormalization(PyObject* self) {
    LEP_RESULT sResult;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_RunRadFfc: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_RunSysFFCNormalization(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_RunRadFfc: Unable to run RAD FFC. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_RunRadFfc: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    Py_RETURN_NONE;
}

///////////////////////////////////////////////////
// LeptonCCI_GetROI is a Function without
// arguments. Returns a tuple:
// (startRow,startCol,endRow,endCol)
///////////////////////////////////////////////////
static PyObject* LeptonCCI_GetROI(PyObject* self) {
    LEP_RAD_ROI_T sROI;
    LEP_RESULT sResult;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROI: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_GetRadSpotmeterRoi(&i2cPort, &sROI);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROI: Unable to get spotmeter ROI. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROI: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    return Py_BuildValue("iiii", sROI.startRow, sROI.startCol, sROI.endRow, sROI.endCol);
}


///////////////////////////////////////////////////
// LeptonCCI_SetROI sets the spot meter ROI with
// a tuple:
// (startRow,startCol,endRow,endCol)
///////////////////////////////////////////////////
static PyObject* LeptonCCI_SetROI(PyObject* self, PyObject* args) {
    LEP_RESULT sResult;
    LEP_RAD_ROI_T sROI;
    int startRow, startCol, endRow, endCol;

    if (!PyArg_ParseTuple(args, "(iiii)", &startCol, &startRow, &endCol, &endRow)){
        sprintf(sError, "LeptonCCI_SetROI: Unable to parese arguments. args: (%i,%i,%i,%i).", startCol, startRow, endCol, endRow);
        return Py_BuildValue("s", sError);
    }
    sROI.startRow = startRow;
    sROI.startCol = startCol;
    sROI.endRow = endRow;
    sROI.endCol = endCol;
    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_SetROI: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_SetRadSpotmeterRoi(&i2cPort, sROI);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_SetROI: Unable to set spotmeter ROI. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_SetROI: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    Py_RETURN_NONE;
}


///////////////////////////////////////////////////
// LeptonCCI_GetROIValues is a Function without
// arguments. Returns a tuple (in degrees celcius):
// (radSpotmeterValue, radSpotmeterMaxValue,
// radSpotmeterMinValue, radSpotmeterPopulation)
///////////////////////////////////////////////////
static PyObject* LeptonCCI_GetROIValues(PyObject* self) {
    LEP_RAD_SPOTMETER_OBJ_KELVIN_T sROIValues;
    LEP_RESULT sResult;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROIValues: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_GetRadSpotmeterObjInKelvinX100(&i2cPort, &sROIValues);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROIValues: Unable to get spotmeter ROI values. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROIValues: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    return Py_BuildValue("fffi", (sROIValues.radSpotmeterValue/100.0)-273.15,
                                 (sROIValues.radSpotmeterMaxValue/100.0)-273.15,
                                 (sROIValues.radSpotmeterMinValue/100.0)-273.15,
                                 sROIValues.radSpotmeterPopulation);
}

///////////////////////////////////////////////////
// LeptonCCI_SetFluxLinearParams is a Function with
// arguments. Takes in a tuple:
// (sceneEmissivity, TBkg, tauWindow, TWindow,
// tauAtm, TAtm, reflWindow, TRefl)
///////////////////////////////////////////////////
static PyObject* LeptonCCI_SetFluxLinearParams(PyObject* self, PyObject* args) {
    LEP_RESULT sResult;
    LEP_RAD_FLUX_LINEAR_PARAMS_T sFLParams;
    float sceneEmissivity, TBkg, tauWindow, TWindow, tauAtm, TAtm, reflWindow, TRefl;

    if (!PyArg_ParseTuple(args, "(ffffffff)", &sceneEmissivity, &TBkg, &tauWindow, &TWindow, &tauAtm, &TAtm, &reflWindow, &TRefl)){
        sprintf(sError, "LeptonCCI_SetFluxLinearParams: Args parsing error. SDK error code %i.", (int)sResult);
        return Py_BuildValue("s", sError);
    }

    sFLParams.sceneEmissivity = CLAMP((int)(sceneEmissivity * 8192.0), 82, 8192);
    sFLParams.TBkgK = CLAMP((int)((TBkg + 273.15) * 100.0), 0, 65535);
    sFLParams.tauWindow = CLAMP((int)(tauWindow * 8192.0), 82, 8192);
    sFLParams.TWindowK = CLAMP((int)((TWindow + 273.15) * 100.0), 0, 65535);
    sFLParams.tauAtm = CLAMP((int)(tauAtm * 8192.0), 82, 8192);
    sFLParams.TAtmK = CLAMP((int)((TAtm + 273.15) * 100.0), 0, 65535);
    sFLParams.reflWindow = CLAMP((int)(reflWindow * 8192.0), 0, 8192-sFLParams.tauWindow);
    sFLParams.TReflK = CLAMP((int)((TRefl + 273.15) * 100.0), 0, 65535);

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_SetFluxLinearParams: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_SetRadFluxLinearParams(&i2cPort, sFLParams);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_SetFluxLinearParams: Unable to set spotmeter ROI. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_SetFluxLinearParams: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    Py_RETURN_NONE;
}



///////////////////////////////////////////////////
// LeptonCCI_GetFluxLinearParams is a Function without
// arguments. Returns a tuple:
// (sceneEmissivity, TBkg, tauWindow, TWindow,
// tauAtm, TAtm, reflWindow, TRefl)
///////////////////////////////////////////////////
static PyObject* LeptonCCI_GetFluxLinearParams(PyObject* self) {
    LEP_RAD_FLUX_LINEAR_PARAMS_T sFLParams;
    LEP_RESULT sResult;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetFluxLinearParams: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_GetRadFluxLinearParams(&i2cPort, &sFLParams);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetFluxLinearParams: Unable to get spotmeter ROI values. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetFluxLinearParams: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    return Py_BuildValue("ffffffff", sFLParams.sceneEmissivity/8192.0,
                                      (sFLParams.TBkgK/100.0)-273.15,
                                      sFLParams.tauWindow/8192.0,
                                      (sFLParams.TWindowK/100.0)-273.15,
                                      sFLParams.tauAtm/8192.0,
                                      (sFLParams.TAtmK/100.0)-273.15,
                                      sFLParams.reflWindow/8192.0,
                                      (sFLParams.TReflK/100.0)-273.15);
}



///////////////////////////////////////////////////
// LeptonCCI_GetAuxTemp is a Function without
// arguments. Returns the camera's aux temperature
// in deg C.
///////////////////////////////////////////////////
static PyObject* LeptonCCI_GetAuxTemp(PyObject* self) {
    LEP_SYS_AUX_TEMPERATURE_CELCIUS_T sAuxTemp;
    LEP_RESULT sResult;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetAuxTemp: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_GetSysAuxTemperatureCelcius(&i2cPort, &sAuxTemp);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetAuxTemp: Unable to get Aux temperature. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetAuxTemp: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    return Py_BuildValue("f", (float)sAuxTemp);
}


///////////////////////////////////////////////////
// LeptonCCI_GetFpaTemp is a Function without
// arguments. Returns the camera's FPA temperature
// in deg C.
///////////////////////////////////////////////////
static PyObject* LeptonCCI_GetFpaTemp(PyObject* self) {
    LEP_SYS_FPA_TEMPERATURE_CELCIUS_T sFpaTemp;
    LEP_RESULT sResult;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetFpaTemp: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_GetSysFpaTemperatureCelcius(&i2cPort, &sFpaTemp);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetFpaTemp: Unable to get FPA temperature. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetFpaTemp: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    return Py_BuildValue("f", (float)sFpaTemp);
}

///////////////////////////////////////////////////
// LeptonCCI_SetRadTLinearEnableState
///////////////////////////////////////////////////
static PyObject* LeptonCCI_SetRadTLinearEnableState(PyObject* self, PyObject* args) {
  LEP_RESULT sResult;
  int enableState;

  if (!PyArg_ParseTuple(args, "i", &enableState)){
      sprintf(sError, "LeptonCCI_SetRadTLinearEnableState: Unable to parese arguments. args: (%i).", enableState);
      return Py_BuildValue("s", sError);
  }
  
  sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
  if(LEP_COMM_OK != sResult)
  {
      sprintf(sError, "LeptonCCI_SetRadTLinearEnableState: Unable to open i2c port. SDK error code %i.", (int)sResult);
      PyErr_SetString(LeptonCCIError, sError);
      return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
  }
  sResult = LEP_SetRadTLinearEnableState(&i2cPort, enableState);
  if(LEP_OK != sResult)
  {
      sprintf(sError, "LeptonCCI_SetRadTLinearEnableState: Unable to set TLinear Enable state. SDK error code %i.", (int)sResult);
      PyErr_SetString(LeptonCCIError, sError);
      return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
  }
  sResult = LEP_ClosePort(&i2cPort);
  if(LEP_OK != sResult)
  {
      sprintf(sError, "LeptonCCI_SetRadTLinearEnableState: Unable to close i2c port. SDK error code %i.", (int)sResult);
      PyErr_SetString(LeptonCCIError, sError);
      return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
  }

  Py_RETURN_NONE;
}

///////////////////////////////////////////////////
// LeptonCCI_GetRadTLinearEnableState
///////////////////////////////////////////////////
static PyObject* LeptonCCI_GetRadTLinearEnableState(PyObject* self) {
    LEP_RESULT sResult;
    int enableState;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetRadTLinearEnableState: Unable to open i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_GetRadTLinearEnableState(&i2cPort, (LEP_RAD_ENABLE_E_PTR)&enableState);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetRadTLinearEnableState: Unable to get TLinear Enable state. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetRadTLinearEnableState: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpretor.
    }

    return Py_BuildValue("i", enableState);
}


///////////////////////////////////////////////////
// LeptonCCI_GetFrameBuffer
// Not really a CCI function but rather uses SPI.
// SPI port is e.g. "/dev/spidev0.0"
///////////////////////////////////////////////////
static PyObject* LeptonCCI_GetFrameBuffer(PyObject* self, PyObject* args) {
    char sSpiPort[16] = "/dev/spidev0.0";
    char sDummy[16];
    PyObject *pyFbList, *item;
    int k = 0;
    int resets = 0;
    int iResult = 0;
    
    if (!PyArg_ParseTuple(args, "s", sDummy))
    {
        sprintf(sError, "LeptonCCI_GetFrameBuffer: Unable to parse arguments.");
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError);
    }
    
    iResult = SpiOpenPort(sSpiPort);
    if (iResult < 0)
    {
        // failed to open SPI port, terminate.
        sprintf(sError, "LeptonCCI_GetFrameBuffer: Unable to open SPI port %s. Error code %i.", sSpiPort, iResult);
        PyErr_SetString(LeptonCCIError, sError);
        return Py_BuildValue("s", sError); // Propagate the error to the Python interpreter.
    }
    
    for(int j=0;j<SPI_PACKETS_PER_FRAME;j++)
    {
        //if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
        iResult = read(miSpiFd, result+sizeof(uint8_t)*SPI_PACKET_SIZE*j, sizeof(uint8_t)*SPI_PACKET_SIZE);
        if (iResult < 0)
        {
            // failed to open SPI port, terminate.
            sprintf(sError, "LeptonCCI_GetFrameBuffer: Unable to read from SPI port. Error code %i.", iResult);
            PyErr_SetString(LeptonCCIError, sError);
            return Py_BuildValue("s", sError); // Propagate the error to the Python interpreter.
        }
        int packetNumber = result[j*SPI_PACKET_SIZE+1];
        if(packetNumber != j)
        {
            j = -1;
            resets += 1;
            usleep(1000);
            //Note: we've selected 750 resets as an arbitrary limit, since there should never be 750 "null" packets between two valid transmissions at the current poll rate
            //By polling faster, developers may easily exceed this count, and the down period between frames may then be flagged as a loss of sync
            if(resets == 750)
            {
                SpiClosePort(0);
                usleep(750000);
                SpiOpenPort(0);
            }
        }
    }
    if(resets >= 30) {
        //printf("[INFO] Got more than 30 resets, done reading.\n");
    }
    
    iResult = SpiClosePort();
    if (iResult < 0)
    {
        // failed to close SPI port
        sprintf(sError, "LeptonCCI_GetFrameBuffer: Unable to close SPI port. Error code %i.", iResult);
        PyErr_SetString(LeptonCCIError, sError);
    }

    // Parse the received data
    frameBuffer = (uint16_t *)result;
    //int row, column;
    uint16_t value;
    uint16_t minValue = 65535;
    uint16_t maxValue = 0;

    uint32_t totalCounts = 0;
    
    pyFbList = PyList_New(LEPTON_WIDTH * LEPTON_HEIGHT);

    for(int i=0;i<SPI_FRAME_SIZE_UINT16;i++) 
    {
        //skip the first 2 uint16_t's of every packet, they're 4 header bytes
        if(i % SPI_PACKET_SIZE_UINT16 < 2)
        {
            continue;
        }
        //flip the MSB and LSB at the last second
        int temp = result[i*2];
        result[i*2] = result[i*2+1];
        result[i*2+1] = temp;

        value = frameBuffer[i];
        totalCounts += value;
        if(value > maxValue)
        {
            maxValue = value;
        }
        if(value < minValue)
        {
            minValue = value;
        }
        //column = i % SPI_PACKET_SIZE_UINT16 - 2;
        //row = i / SPI_PACKET_SIZE_UINT16 ;
        
        item = PyLong_FromLong((long)frameBuffer[i]);
        PyList_SetItem(pyFbList, k, item);
        k += 1;
    }
    
    mlFbMaxValue = maxValue;
    mlFbMinValue = minValue;
    
    return pyFbList;
}


///////////////////////////////////////////////////
// Helper function for SPI -- SpiOpenPort
// Error return values:
// -1: Error - Could not open SPI device
// -2: Could not set SPIMode (WR)...ioctl fail
// -3: Could not set SPIMode (RD)...ioctl fail
// -4: Could not set SPI bitsPerWord (WR)...ioctl fail
// -5: Could not set SPI bitsPerWord(RD)...ioctl fail
// -6: Could not set SPI speed (WR)...ioctl fail
// -7: Could not set SPI speed (RD)...ioctl fail
///////////////////////////////////////////////////
static int SpiOpenPort (char *sPort)
{
	int status_value = -1;

	//----- SET SPI MODE -----
	//SPI_MODE_0 (0,0)  CPOL=0 (Clock Idle low level), CPHA=0 (SDO transmit/change edge active to idle)
	//SPI_MODE_1 (0,1)  CPOL=0 (Clock Idle low level), CPHA=1 (SDO transmit/change edge idle to active)
	//SPI_MODE_2 (1,0)  CPOL=1 (Clock Idle high level), CPHA=0 (SDO transmit/change edge active to idle)
	//SPI_MODE_3 (1,1)  CPOL=1 (Clock Idle high level), CPHA=1 (SDO transmit/change edge idle to active)
	unsigned char spi_mode = SPI_MODE_3;

	//----- SET BITS PER WORD -----
	unsigned char spi_bitsPerWord = 8;

	//----- SET SPI BUS SPEED -----
	unsigned int spi_speed = 24000000;				//24000000 = 24MHz (1uS per bit)
    
	miSpiFd = open(sPort, O_RDWR | O_NONBLOCK);
	if (miSpiFd < 0)
	{
		//printf("Error - Could not open SPI device %s", sPortName);
		//exit(1);
        return -1;
	}

	status_value = ioctl(miSpiFd, SPI_IOC_WR_MODE, &spi_mode);
	if(status_value < 0)
	{
		//printf("Could not set SPIMode (WR)...ioctl fail");
		//exit(1);
        return -2;
	}

	status_value = ioctl(miSpiFd, SPI_IOC_RD_MODE, &spi_mode);
	if(status_value < 0)
	{
		//printf("Could not set SPIMode (RD)...ioctl fail");
		//exit(1);
        return -3;
	}

	status_value = ioctl(miSpiFd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
	if(status_value < 0)
	{
		//printf("Could not set SPI bitsPerWord (WR)...ioctl fail");
		//exit(1);
        return -4;
	}

	status_value = ioctl(miSpiFd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
	if(status_value < 0)
	{
		//printf("Could not set SPI bitsPerWord(RD)...ioctl fail");
		//exit(1);
        return -5;
	}

	status_value = ioctl(miSpiFd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
	if(status_value < 0)
	{
		//printf("Could not set SPI speed (WR)...ioctl fail");
		//exit(1);
        return -6;
	}

	status_value = ioctl(miSpiFd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
	if(status_value < 0)
	{
		//printf("Could not set SPI speed (RD)...ioctl fail");
		//exit(1);
        return -7;
	}
	return(status_value);
}

///////////////////////////////////////////////////
// Helper function for SPI -- SpiClosePort
// Error return values:
// -1: Error - Could not close SPI device
///////////////////////////////////////////////////
static int SpiClosePort()
{
	int status_value = -1;
    
    if (miSpiFd > 0)
    {
        status_value = close(miSpiFd);
    }
    else
    {
        status_value = 100;
    }

	return(status_value);
}
///////////////////////////////////////////////////


// Method mapping table
// Method definition object for this extension, these argumens mean:
// ml_name: The name of the method
// ml_meth: Function pointer to the method implementation
// ml_flags: Flags indicating special features of this method, such as
//          accepting arguments, accepting keyword arguments, being a
//          class method, or being a static method of a class.
// ml_doc:  Contents of this method's docstring
static PyMethodDef LeptonCCI_methods[] = {
    {"RunRadFfc", (PyCFunction)LeptonCCI_RunRadFfc, METH_NOARGS, NULL},
    {"RunSysFFCNormalization", (PyCFunction)LeptonCCI_RunSysFFCNormalization, METH_NOARGS, NULL},
    {"GetROI", (PyCFunction)LeptonCCI_GetROI, METH_NOARGS, NULL},
    {"SetROI", (PyCFunction)LeptonCCI_SetROI, METH_VARARGS, NULL},
    {"GetROIValues", (PyCFunction)LeptonCCI_GetROIValues, METH_NOARGS, NULL},
    {"SetFluxLinearParams", (PyCFunction)LeptonCCI_SetFluxLinearParams, METH_VARARGS, NULL},
    {"GetFluxLinearParams", (PyCFunction)LeptonCCI_GetFluxLinearParams, METH_NOARGS, NULL},
    {"GetAuxTemp", (PyCFunction)LeptonCCI_GetAuxTemp, METH_NOARGS, NULL},
    {"GetFpaTemp", (PyCFunction)LeptonCCI_GetFpaTemp, METH_NOARGS, NULL},
    {"SetRadTLinearEnableState", (PyCFunction)LeptonCCI_SetRadTLinearEnableState, METH_VARARGS, NULL},
    {"GetRadTLinearEnableState", (PyCFunction)LeptonCCI_GetRadTLinearEnableState, METH_NOARGS, NULL},
    {"GetFrameBuffer", (PyCFunction)LeptonCCI_GetFrameBuffer, METH_VARARGS, NULL},
    {NULL, NULL, 0, NULL}
};

// Module definition
// The arguments of this structure tell Python what to call your extension,
// what it's methods are and where to look for it's method definitions
static struct PyModuleDef LeptonCCI_definition = {
    PyModuleDef_HEAD_INIT,
    "LeptonCCI",
    "A Python wrapper around Flir Lepton embedded SDK.",
    -1,
    LeptonCCI_methods
};


// Module initialization
// Python calls this function when importing your extension. It is important
// that this function is named PyInit_[[your_module_name]] exactly, and matches
// the name keyword argument in setup.py's setup() call.
PyMODINIT_FUNC PyInit_LeptonCCI(void) {
    Py_Initialize();
    return PyModule_Create(&LeptonCCI_definition);
}

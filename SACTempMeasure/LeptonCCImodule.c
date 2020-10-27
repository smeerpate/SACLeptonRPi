#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <stdio.h>
#include <unistd.h>

#include <LEPTON_SDK.h>
#include <LEPTON_SYS.h>
#include <LEPTON_OEM.h>
#include <LEPTON_Types.h>
#include <LEPTON_RAD.h>

PyObject* LeptonCCIError;
LEP_CAMERA_PORT_DESC_T i2cPort;
char sError[128];

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
        return NULL; // Propagate the error to the Python interpretor.
    }
    sResult = LEP_RunRadFFC(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_RunRadFfc: Unable to run RAD FFC. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return NULL; // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_RunRadFfc: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return NULL; // Propagate the error to the Python interpretor.
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
        return NULL; // Propagate the error to the Python interpretor.
    }
    sResult = LEP_GetRadSpotmeterRoi(&i2cPort, &sROI);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROI: Unable to get spotmeter ROI. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return NULL; // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROI: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return NULL; // Propagate the error to the Python interpretor.
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

    if (!PyArg_ParseTuple(args, "(iiii)", &startRow, &startCol, &endRow, &endCol)){
        return NULL;
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
        return NULL; // Propagate the error to the Python interpretor.
    }
    sResult = LEP_SetRadSpotmeterRoi(&i2cPort, sROI);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_SetROI: Unable to set spotmeter ROI. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return NULL; // Propagate the error to the Python interpretor.
    }
    sResult = LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_SetROI: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return NULL; // Propagate the error to the Python interpretor.
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
        return NULL; // Propagate the error to the Python interpretor.
    }
    sResult = LEP_GetRadSpotmeterObjInKelvinX100(&i2cPort, &sROIValues);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROIValues: Unable to get spotmeter ROI values. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return NULL; // Propagate the error to the Python interpretor.
    }
    LEP_ClosePort(&i2cPort);
    if(LEP_OK != sResult)
    {
        sprintf(sError, "LeptonCCI_GetROIValues: Unable to close i2c port. SDK error code %i.", (int)sResult);
        PyErr_SetString(LeptonCCIError, sError);
        return NULL; // Propagate the error to the Python interpretor.
    }

    return Py_BuildValue("fffi", (sROIValues.radSpotmeterValue/100.0)-273.15,
                                 (sROIValues.radSpotmeterMaxValue/100.0)-273.15,
                                 (sROIValues.radSpotmeterMinValue/100.0)-273.15,
                                 sROIValues.radSpotmeterPopulation);
}


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
    {"GetROI", (PyCFunction)LeptonCCI_GetROI, METH_NOARGS, NULL},
    {"SetROI", (PyCFunction)LeptonCCI_SetROI, METH_VARARGS, NULL},
    {"GetROIValues", (PyCFunction)LeptonCCI_GetROIValues, METH_NOARGS, NULL},
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

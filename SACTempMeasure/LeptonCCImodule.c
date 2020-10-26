#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <stdio.h>
#include <unistd.h>

#include <LEPTON_SDK.h>
#include <LEPTON_SYS.h>
#include <LEPTON_OEM.h>
#include <LEPTON_Types.h>

PyObject* LeptonCCIError;
LEP_CAMERA_PORT_DESC_T i2cPort;

// Functions
///////////////////////////////////////////////////
// LeptonCCI_RunRadFfc is a Function without
// arguments.
///////////////////////////////////////////////////
static PyObject* LeptonCCI_RunRadFfc(PyObject* self) {
    LEP_RESULT sResult;

    sResult = LEP_OpenPort(1, LEP_CCI_TWI, 400, &i2cPort);
    if(LEP_COMM_OK != sResult)
        return Py_BuildValue("i", sResult);

    sResult = LEP_RunRadFFC(&i2cPort);
    if(LEP_OK != sResult)
        return Py_BuildValue("i", sResult);

    sResult = LEP_ClosePort(&i2cPort);
    return Py_BuildValue("i", sResult);
}


// Method mapping table
static PyMethodDef LeptonCCI_methods[] = {
    {"RunRadFfc", (PyFunction)LeptonCCI_RunRadFfc, METH_NOARGS, NULL},
    {NULL, NULL, 0, NULL}
};

// Initialisation function. Called when Python interpreter loads the module.
PyMODINIT_FUNC initLeptonCCI() {
    Py_InitModule3("RunRadFfc", LeptonCCI_methods, "Runs the radiometry flat field calibration.");
}

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
// Method definition object for this extension, these argumens mean:
// ml_name: The name of the method
// ml_meth: Function pointer to the method implementation
// ml_flags: Flags indicating special features of this method, such as
//          accepting arguments, accepting keyword arguments, being a
//          class method, or being a static method of a class.
// ml_doc:  Contents of this method's docstring
static PyMethodDef LeptonCCI_methods[] = {
    {"RunRadFfc", (PyCFunction)LeptonCCI_RunRadFfc, METH_NOARGS, NULL},
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

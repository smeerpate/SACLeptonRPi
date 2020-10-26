#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <stdio.h>
#include <unistd.h>

#include <LEPTON_SDK.h>
#include <LEPTON_SYS.h>
#include <LEPTON_OEM.h>
#include <LEPTON_Types.h>

PyObject* LeptonCCIError;

// Functions
static PyObject* LeptonCCI_RunRadFfc(PyObject* self)
{
    
    Py_RETURN_NONE;
}


// Method mapping table

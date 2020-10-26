# Install with: sudo python3 setup.py install

from distutils.core import setup, Extension

module1 = Extension('LeptonCCI',
                    include_dirs = ['/opt/lepton/leptonSDKEmb32PUB'],
                    libraries = ['LEPTON_SDK'],
                    library_dirs = ['/opt/lepton/leptonSDKEmb32PUB/Debug'],
                    sources = ['LeptonCCImodule.c'])

setup(name='LeptonCCI',
      version='1.0',
      ext_modules=[module1])

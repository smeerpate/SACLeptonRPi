#!/usr/bin/env python

import numpy as np
import ctypes
import struct
import time

# relative imports in Python3 must be explicit
from ioctl_numbers import _IOR, _IOW
from fcntl import ioctl

########################################################
# SPI constants
########################################################
SPI_IOC_MAGIC   = ord("k")
SPI_IOC_RD_MODE          = _IOR(SPI_IOC_MAGIC, 1, "=B")
SPI_IOC_WR_MODE          = _IOW(SPI_IOC_MAGIC, 1, "=B")
SPI_IOC_RD_LSB_FIRST     = _IOR(SPI_IOC_MAGIC, 2, "=B")
SPI_IOC_WR_LSB_FIRST     = _IOW(SPI_IOC_MAGIC, 2, "=B")
SPI_IOC_RD_BITS_PER_WORD = _IOR(SPI_IOC_MAGIC, 3, "=B")
SPI_IOC_WR_BITS_PER_WORD = _IOW(SPI_IOC_MAGIC, 3, "=B")
SPI_IOC_RD_MAX_SPEED_HZ  = _IOR(SPI_IOC_MAGIC, 4, "=I")
SPI_IOC_WR_MAX_SPEED_HZ  = _IOW(SPI_IOC_MAGIC, 4, "=I")
SPI_CPHA   = 0x01                 # /* clock phase */
SPI_CPOL   = 0x02                 # /* clock polarity */
SPI_MODE_0 = (0|0)                # /* (original MicroWire) */
SPI_MODE_1 = (0|SPI_CPHA)
SPI_MODE_2 = (SPI_CPOL|0)
SPI_MODE_3 = (SPI_CPOL|SPI_CPHA)


########################################################
# Main I2C communication class for Lepton
# commissioning.
# Default I2C channel is 1.
########################################################
class LeptonCommissioning(object):
	# The Lepton camera's device address
	LEP_I2C_DEVICE_ADDRESS          = 0x2A
	# Block Data Buffers
	LEP_DATA_BUFFER_0_BASE_ADDR     = 0xF800
	LEP_DATA_BUFFER_1_BASE_ADDR     = 0xFC00
	# The Lepton I2C Registers Sub-Addresses
	LEP_I2C_REG_BASE_ADDR           = 0x0000
	# Host On Switch when camera is in stand by of off
	LEP_I2C_POWER_REG               = (LEP_I2C_REG_BASE_ADDR + 0x0000)
	# Host Command Interface over I2C
	LEP_I2C_STATUS_REG              = (LEP_I2C_REG_BASE_ADDR + 0x0002 )
	LEP_I2C_COMMAND_REG             = (LEP_I2C_REG_BASE_ADDR + 0x0004 )
	LEP_I2C_DATA_LENGTH_REG         = (LEP_I2C_REG_BASE_ADDR + 0x0006 )
	LEP_I2C_DATA_0_REG              = (LEP_I2C_REG_BASE_ADDR + 0x0008 )
	LEP_I2C_DATA_1_REG              = (LEP_I2C_REG_BASE_ADDR + 0x000A )
	LEP_I2C_DATA_2_REG              = (LEP_I2C_REG_BASE_ADDR + 0x000C )
	LEP_I2C_DATA_3_REG              = (LEP_I2C_REG_BASE_ADDR + 0x000E )
	LEP_I2C_DATA_4_REG              = (LEP_I2C_REG_BASE_ADDR + 0x0010 )
	LEP_I2C_DATA_5_REG              = (LEP_I2C_REG_BASE_ADDR + 0x0012 )
	LEP_I2C_DATA_6_REG              = (LEP_I2C_REG_BASE_ADDR + 0x0014 )
	LEP_I2C_DATA_7_REG              = (LEP_I2C_REG_BASE_ADDR + 0x0016 )
	LEP_I2C_DATA_8_REG              = (LEP_I2C_REG_BASE_ADDR + 0x0018 )
	LEP_I2C_DATA_9_REG              = (LEP_I2C_REG_BASE_ADDR + 0x001A )
	LEP_I2C_DATA_10_REG             = (LEP_I2C_REG_BASE_ADDR + 0x001C )
	LEP_I2C_DATA_11_REG             = (LEP_I2C_REG_BASE_ADDR + 0x001E )
	LEP_I2C_DATA_12_REG             = (LEP_I2C_REG_BASE_ADDR + 0x0020 )
	LEP_I2C_DATA_13_REG             = (LEP_I2C_REG_BASE_ADDR + 0x0022 )
	LEP_I2C_DATA_14_REG             = (LEP_I2C_REG_BASE_ADDR + 0x0024 )
	LEP_I2C_DATA_15_REG             = (LEP_I2C_REG_BASE_ADDR + 0x0026 )
	LEP_I2C_DATA_CRC_REG            = (LEP_I2C_REG_BASE_ADDR + 0x0028 )
	LEP_I2C_DATA_BUFFER_0           = (LEP_DATA_BUFFER_0_BASE_ADDR )
	LEP_I2C_DATA_BUFFER_0_END       = (LEP_DATA_BUFFER_0_BASE_ADDR + 0x03FF )
	LEP_I2C_DATA_BUFFER_0_LENGTH    =  0x400
	LEP_I2C_DATA_BUFFER_1           = (LEP_DATA_BUFFER_1_BASE_ADDR )
	LEP_I2C_DATA_BUFFER_1_END       = (LEP_DATA_BUFFER_1_BASE_ADDR + 0x03FF )
	LEP_I2C_DATA_BUFFER_1_LENGTH    = 0x400
	LEP_I2C_STATUS_BUSY_BIT_MASK    = 0x0001 # Bit 0 is the Busy Bit
	# error codes
	LEP_OK                           = 0       # Camera ok
	LEP_COMM_OK                      = LEP_OK, # Camera comm ok (same as LEP_OK)
	LEP_ERROR                        = -1,    # Camera general error
	LEP_NOT_READY                    = -2,    # Camera not ready error
	LEP_RANGE_ERROR                  = -3,    # Camera range error
	LEP_CHECKSUM_ERROR               = -4,    # Camera checksum error
	LEP_BAD_ARG_POINTER_ERROR        = -5,    # Camera Bad argument  error
	LEP_DATA_SIZE_ERROR              = -6,    # Camera byte count error
	LEP_UNDEFINED_FUNCTION_ERROR     = -7,    # Camera undefined function error
	LEP_FUNCTION_NOT_SUPPORTED       = -8,    # Camera function not yet supported error
	# OTP access errors
	LEP_OTP_WRITE_ERROR              = -15,   #!< Camera OTP write error
	LEP_OTP_READ_ERROR		 = -16,   # double bit error detected (uncorrectible)
	LEP_OTP_NOT_PROGRAMMED_ERROR     = -18,   # Flag read as non-zero
	# I2C Errors
	LEP_ERROR_I2C_BUS_NOT_READY      = -20,   # I2C Bus Error - Bus Not Avaialble
	LEP_ERROR_I2C_BUFFER_OVERFLOW    = -22,   # I2C Bus Error - Buffer Overflow
	LEP_ERROR_I2C_ARBITRATION_LOST   = -23,   # I2C Bus Error - Bus Arbitration Lost
	LEP_ERROR_I2C_BUS_ERROR          = -24,   # I2C Bus Error - General Bus Error
	LEP_ERROR_I2C_NACK_RECEIVED      = -25,   # I2C Bus Error - NACK Received
	LEP_ERROR_I2C_FAIL               = -26,   # I2C Bus Error - General Failure
	# Processing Errors
	LEP_DIV_ZERO_ERROR               = -80,   # Attempted div by zero
	# Comm Errors
	LEP_COMM_PORT_NOT_OPEN           = -101,  # Comm port not open
	LEP_COMM_INVALID_PORT_ERROR      = -102,  # Comm port no such port error
	LEP_COMM_RANGE_ERROR             = -103,  # Comm port range error
	LEP_ERROR_CREATING_COMM          = -104,  # Error creating comm
	LEP_ERROR_STARTING_COMM          = -105,  # Error starting comm
	LEP_ERROR_CLOSING_COMM           = -106,  # Error closing comm
	LEP_COMM_CHECKSUM_ERROR          = -107,  # Comm checksum error
	LEP_COMM_NO_DEV                  = -108,  # No comm device
	LEP_TIMEOUT_ERROR                = -109,  # Comm timeout error
	LEP_COMM_ERROR_WRITING_COMM      = -110,  # Error writing comm
	LEP_COMM_ERROR_READING_COMM      = -111,  # Error reading comm
	LEP_COMM_COUNT_ERROR             = -112,  # Comm byte count error
	# Other Errors
	LEP_OPERATION_CANCELED           = -126,  # Camera operation canceled
	LEP_UNDEFINED_ERROR_CODE         = -127   # Undefined error

	def __init__(self, i2c_channel = 1):
		self.__i2cBus = smbus.SMBus(i2c_channel)

########################################################
# Main SPI communication class.
########################################################
class Lepton(object):
  """Communication class for FLIR Lepton module on SPI
  Args:
    spi_dev (str): Location of SPI device node. Default '/dev/spidev0.0'.
  """
  ROWS = 60
  COLS = 80
  VOSPI_FRAME_SIZE = COLS + 2
  VOSPI_FRAME_SIZE_BYTES = VOSPI_FRAME_SIZE * 2
  MODE = SPI_MODE_3
  BITS = 8
  SPEED = 18000000
  SPIDEV_MESSAGE_LIMIT = 24

  def __init__(self, spi_dev = "/dev/spidev0.0"):
    self.__spi_dev = spi_dev
    self.__txbuf = np.zeros(Lepton.VOSPI_FRAME_SIZE, dtype=np.uint16)
    # struct spi_ioc_transfer {
    #   __u64     tx_buf;
    #   __u64     rx_buf;
    #   __u32     len;
    #   __u32     speed_hz;
    #   __u16     delay_usecs;
    #   __u8      bits_per_word;
    #   __u8      cs_change;
    #   __u32     pad;
    # };
    self.__xmit_struct = struct.Struct("=QQIIHBBI")
    self.__msg_size = self.__xmit_struct.size
    self.__xmit_buf = np.zeros((self.__msg_size * Lepton.ROWS), dtype=np.uint8)
    self.__msg = _IOW(SPI_IOC_MAGIC, 0, self.__xmit_struct.format)
    self.__capture_buf = np.zeros((Lepton.ROWS, Lepton.VOSPI_FRAME_SIZE, 1), dtype=np.uint16)

    for i in range(Lepton.ROWS):
      self.__xmit_struct.pack_into(self.__xmit_buf, i * self.__msg_size,
        self.__txbuf.ctypes.data,                                            #   __u64     tx_buf;
        self.__capture_buf.ctypes.data + Lepton.VOSPI_FRAME_SIZE_BYTES * i,  #   __u64     rx_buf;
        Lepton.VOSPI_FRAME_SIZE_BYTES,                                      #   __u32     len;
        Lepton.SPEED,                                                       #   __u32     speed_hz;
        0,                                                                  #   __u16     delay_usecs;
        Lepton.BITS,                                                        #   __u8      bits_per_word;
        1,                                                                  #   __u8      cs_change;
        0)                                                                  #   __u32     pad;

  def __enter__(self):
    # "In Python 3 the only way to open /dev/tty under Linux appears to be 1) in binary mode and 2) with buffering disabled."
    self.__handle = open(self.__spi_dev, "wb+", buffering=0)
    ioctl(self.__handle, SPI_IOC_RD_MODE, struct.pack("=B", Lepton.MODE))
    ioctl(self.__handle, SPI_IOC_WR_MODE, struct.pack("=B", Lepton.MODE))
    ioctl(self.__handle, SPI_IOC_RD_BITS_PER_WORD, struct.pack("=B", Lepton.BITS))
    ioctl(self.__handle, SPI_IOC_WR_BITS_PER_WORD, struct.pack("=B", Lepton.BITS))
    ioctl(self.__handle, SPI_IOC_RD_MAX_SPEED_HZ, struct.pack("=I", Lepton.SPEED))
    ioctl(self.__handle, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("=I", Lepton.SPEED))
    #print(self.__handle)
    return self


  def __exit__(self, type, value, tb):
    self.__handle.close()


  @staticmethod
  def capture_segment(handle, xs_buf, xs_size, capture_buf):
    messages = Lepton.ROWS
    #print(messages)
    iow = _IOW(SPI_IOC_MAGIC, 0, xs_size)
    ioctl(handle, iow, xs_buf, True)
    while (capture_buf[0] & 0x000f) == 0x000f: # byteswapped 0x0f00
      ioctl(handle, iow, xs_buf, True)
    messages -= 1
    # NB: the default spidev bufsiz is 4096 bytes so that's where the 24 message limit comes from: 4096 / Lepton.VOSPI_FRAME_SIZE_BYTES = 24.97...
    # This 24 message limit works OK, but if you really need to optimize the read speed here, this hack is for you:
    # The limit can be changed when spidev is loaded, but since it is compiled statically into newer raspbian kernels, that means
    # modifying the kernel boot args to pass this option. This works too:
    #   $ sudo chmod 666 /sys/module/spidev/parameters/bufsiz
    #   $ echo 65536 > /sys/module/spidev/parameters/bufsiz
    # Then Lepton.SPIDEV_MESSAGE_LIMIT of 24 can be raised to 59
    while messages > 0:
      if messages > Lepton.SPIDEV_MESSAGE_LIMIT:
        count = Lepton.SPIDEV_MESSAGE_LIMIT
      else:
        count = messages
      iow = _IOW(SPI_IOC_MAGIC, 0, xs_size * count)
      ret = ioctl(handle, iow, xs_buf[xs_size * (60 - messages):], True)
      if ret < 1:
        raise IOError("can't send {0} spi messages ({1})".format(60, ret))
      messages -= count


  def capture(self, data_buffer = None, log_time = False, debug_print = False, retry_reset = True):
    """Capture a frame of data.
    Captures 80x60 uint16 array of non-normalized (raw 12-bit) data. Returns that frame and a frame_id (which
    is currently just the sum of all pixels). The Lepton will return multiple, identical frames at a rate of up
    to ~27 Hz, with unique frames at only ~9 Hz, so the frame_id can help you from doing additional work
    processing duplicate frames.

    Args:
      data_buffer (numpy.ndarray): Optional. If specified, should be ``(60,80,1)`` with `dtype`=``numpy.uint16``.

    Returns:
      tuple consisting of (data_buffer, frame_id)
    """
    start = time.time()
    #print(start)
    if data_buffer is None:
      data_buffer = np.ndarray((Lepton.ROWS, Lepton.COLS, 1), dtype=np.uint16)
    elif data_buffer.ndim < 2 or data_buffer.shape[0] < Lepton.ROWS or data_buffer.shape[1] < Lepton.COLS or data_buffer.itemsize < 2:
      raise Exception("Provided input array not large enough")
    
    self.__handle = open(self.__spi_dev, "wb+", buffering=0)
    #print(self.__handle)
    ioctl(self.__handle, SPI_IOC_RD_MODE, struct.pack("=B", Lepton.MODE))
    ioctl(self.__handle, SPI_IOC_WR_MODE, struct.pack("=B", Lepton.MODE))
    ioctl(self.__handle, SPI_IOC_RD_BITS_PER_WORD, struct.pack("=B", Lepton.BITS))
    ioctl(self.__handle, SPI_IOC_WR_BITS_PER_WORD, struct.pack("=B", Lepton.BITS))
    ioctl(self.__handle, SPI_IOC_RD_MAX_SPEED_HZ, struct.pack("=I", Lepton.SPEED))
    ioctl(self.__handle, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("=I", Lepton.SPEED))
    
    while True:
      Lepton.capture_segment(self.__handle, self.__xmit_buf, self.__msg_size, self.__capture_buf[0])
      if retry_reset and (self.__capture_buf[20, 0] & 0xFF0F) != 0x1400: # make sure that this is a well-formed frame, should find line 20 here
        # Leave chip select deasserted for at least 185 ms to reset
        if debug_print:
          print("Garbage frame number reset waiting...")
        time.sleep(0.185)
      else:
        break

    self.__capture_buf.byteswap(True)
    data_buffer[:,:] = self.__capture_buf[:,2:]
    end = time.time()

    if debug_print:
      print("---")
      for i in range(Lepton.ROWS):
        fid = self.__capture_buf[i, 0, 0]
        crc = self.__capture_buf[i, 1, 0]
        fnum = fid & 0xFFF
        print("0x{0:04x} 0x{1:04x} : Row {2:2} : crc={1}".format(fid, crc, fnum))
      print("---")

    if log_time:
      print("frame processed int {0}s, {1}hz".format(end-start, 1.0/(end-start)))

    # TODO: turn on telemetry to get real frame id, sum on this array is fast enough though (< 500us)
    return data_buffer, data_buffer.sum()

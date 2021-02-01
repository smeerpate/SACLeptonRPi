#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <LEPTON_SDK.h>
#include <LEPTON_SYS.h>
#include <LEPTON_OEM.h>
#include <LEPTON_RAD.h>
#include <LEPTON_Types.h>


#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <sys/stat.h> // for S_IREAD | S_IWRITE

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
  
void LeptonCCI_GetFrameBuffer(void);
//int SpiOpenSACPort (char *sPort);
int SpiOpenSACPort (void);
int SpiCloseSACPort(void);
void SACPrinter(void);


bool biIsConnected = false;
LEP_CAMERA_PORT_DESC_T i2cPort;
int miSpiFd = 0xFFFF;
uint8_t baRawSensorData[SPI_PACKET_SIZE*SPI_PACKETS_PER_FRAME];
uint16_t *frameBuffer;

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

 //   sResult = LEP_RunRadFFC(&i2cPort);
 //   printf("sResult Run RAD FFC: %d\n", sResult);

    sResult = LEP_GetRadSpotmeterObjInKelvinX100(&i2cPort, &sSpotMeter);
    printf("Spotmeter Value: %i, Max: %i, Min: %i, Population: %i.\n", sSpotMeter.radSpotmeterValue, sSpotMeter.radSpotmeterMaxValue, sSpotMeter.radSpotmeterMinValue, sSpotMeter.radSpotmeterPopulation);
    printf("sResult Get Spotmeter obj: %d\n", sResult);


//    sResult = LEP_RunSysFFCNormalization(&i2cPort);
//    printf("sResult Run FFC: %d\n", sResult);

    sResult = LEP_ClosePort(&i2cPort);
    printf("sResult Close i2c Port: %d\n", sResult);
	
	LeptonCCI_GetFrameBuffer();
	int iResult = SpiCloseSACPort();
	printf("iResult Close SPI Port: %d\n", iResult);
}




///////////////////////////////////////////////////
// LeptonCCI_GetFrameBuffer
// Not really a CCI function but rather uses SPI.
// SPI port is e.g. "/dev/spidev0.0"
///////////////////////////////////////////////////
void LeptonCCI_GetFrameBuffer(void)
{
    //char sSpiPort[16] = "/dev/spidev0.0";
    char sDummy[16];
    int k = 0;
    int resets = 0;
    int iResult = 0;
	
	unsigned char bSpiMode = SPI_MODE_3;
    
    //iResult = SpiOpenSACPort(sSpiPort);
	//printf("[INFO] Trying to open SPI port...\n");
	//miSpiFd = open("/dev/spidev0.0", O_RDWR, S_IREAD | S_IWRITE);
	//printf("[INFO] File descriptor is %x. Last error was: %s\n", miSpiFd, strerror(errno));
	//iResult = ioctl(miSpiFd, SPI_IOC_WR_MODE, &bSpiMode);
	//printf("[INFO] ioctl is %x. last error was: %s\n", iResult, strerror(errno));
	//SACPrinter();
	
	printf("[INFO] Trying to configure SPI port...\n");
	iResult = SpiOpenSACPort();
	
	
    if (iResult < 0)
    {
        // failed to open SPI port, terminate.
        //printf("[ERROR] LeptonCCI_GetFrameBuffer: Unable to open SPI port %s. Error code %i.", sSpiPort, iResult);
		printf("[ERROR] LeptonCCI_GetFrameBuffer: Unable to open SPI port. Error code %i.\n", iResult);
		return;
    }
    
    for(int j=0;j<SPI_PACKETS_PER_FRAME;j++)
    {
        //if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
        iResult = read(miSpiFd, baRawSensorData+sizeof(uint8_t)*SPI_PACKET_SIZE*j, sizeof(uint8_t)*SPI_PACKET_SIZE);
        if (iResult < 0)
        {
            // failed to open SPI port, terminate.
            printf("[ERROR] LeptonCCI_GetFrameBuffer: Unable to read from SPI port. Error code %i.\n", iResult);
            return;
        }
        int packetNumber = baRawSensorData[j*SPI_PACKET_SIZE+1]  + (baRawSensorData[j*SPI_PACKET_SIZE] << 8);
        if((packetNumber & 0x00FF) != j)
		{
			if((packetNumber & 0x0F00) == 0x0F00)
			{
				printf("[INFO] Got dummy data.\n");
			}
				
			j = -1;
            resets += 1;
            usleep(1000);
            //Note: we've selected 750 resets as an arbitrary limit, since there should never be 750 "null" packets between two valid transmissions at the current poll rate
            //By polling faster, developers may easily exceed this count, and the down period between frames may then be flagged as a loss of sync
            if(resets == 750)
            {
                SpiCloseSACPort();
                usleep(750000);
                //SpiOpenSACPort(sSpiPort);
				SpiOpenSACPort();
            }
        }
		else
		{
			printf("[INFO] Got valid data: packet# %d\n", packetNumber);
		}
		//printf("[INFO] Copying framebuffer (%d,0x%x)\n", j, packetNumber);
		lseek(miSpiFd, 0, SEEK_SET);
    }
	
    if(resets >= 30)
	{
        printf("[INFO] Got more than 30 resets, done reading.\n");
    }
    
    iResult = SpiCloseSACPort();
    if (iResult < 0)
    {
        // failed to close SPI port
        printf("[ERROR] LeptonCCI_GetFrameBuffer: Unable to close SPI port. Error code %i.\n", iResult);
        return;
    }

    // Parse the received data
    frameBuffer = (uint16_t *)baRawSensorData;
    //int row, column;
    uint16_t value;
    uint16_t minValue = 65535;
    uint16_t maxValue = 0;

    uint32_t totalCounts = 0;

    for(int i=0;i<SPI_FRAME_SIZE_UINT16;i++) 
    {
        //skip the first 2 uint16_t's of every packet, they're 4 header bytes
        if(i % SPI_PACKET_SIZE_UINT16 < 2)
        {
            continue;
        }
        //flip the MSB and LSB at the last second
        int temp = baRawSensorData[i*2];
        baRawSensorData[i*2] = baRawSensorData[i*2+1];
        baRawSensorData[i*2+1] = temp;

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
       
    }
}

void SACPrinter(void)
{
	printf("[INFO] Checking function calls");
	return;
}


///////////////////////////////////////////////////
// Helper function for SPI -- SpiOpenSACPort
// Error return values:
// -1: Error - Could not open SPI device
// -2: Could not set SPIMode (WR)...ioctl fail
// -3: Could not set SPIMode (RD)...ioctl fail
// -4: Could not set SPI bitsPerWord (WR)...ioctl fail
// -5: Could not set SPI bitsPerWord(RD)...ioctl fail
// -6: Could not set SPI speed (WR)...ioctl fail
// -7: Could not set SPI speed (RD)...ioctl fail
///////////////////////////////////////////////////
//int SpiOpenSACPort (char *sPort)
int SpiOpenSACPort(void)
{
	printf("[INFO] Opening SPI device /dev/spidev0.0 ...\n");
	
	int iStatusValue = -999;

	//----- SET SPI MODE -----
	//bSpiMode_0 (0,0)  CPOL=0 (Clock Idle low level), CPHA=0 (SDO transmit/change edge active to idle)
	//bSpiMode_1 (0,1)  CPOL=0 (Clock Idle low level), CPHA=1 (SDO transmit/change edge idle to active)
	//bSpiMode_2 (1,0)  CPOL=1 (Clock Idle high level), CPHA=0 (SDO transmit/change edge active to idle)
	//bSpiMode_3 (1,1)  CPOL=1 (Clock Idle high level), CPHA=1 (SDO transmit/change edge idle to active)
	unsigned char bSpiMode = SPI_MODE_3;

	//----- SET BITS PER WORD -----
	unsigned char bSpiBitsPerWord = 8;

	//----- SET SPI BUS SPEED -----
	unsigned int uiSpiSpeed = 24000000;				//24000000 = 24MHz (1uS per bit)
    
	//miSpiFd = open(sPort, O_RDWR | O_NONBLOCK);
	//miSpiFd = open("/dev/spidev0.0", O_RDWR | O_NONBLOCK);
	miSpiFd = open("/dev/spidev0.0", O_RDWR | O_TRUNC | O_NONBLOCK , S_IREAD | S_IWRITE);
	printf("[INFO] File descriptor is %x. Last error was: %s\n", miSpiFd, strerror(errno));
	if (miSpiFd < 0)
	{
		//printf("Error - Could not open SPI device %s", sPortName);
		//exit(1);
        iStatusValue += -1;
	}

	iStatusValue = ioctl(miSpiFd, SPI_IOC_WR_MODE, &bSpiMode);
	printf("[INFO] ioctl->SPI_IOC_WR_MODE = %d\n", iStatusValue);
	if(iStatusValue < 0)
	{
		//printf("Could not set SPIMode (WR)...ioctl fail");
		//exit(1);
       iStatusValue += -2;
	}

	iStatusValue = ioctl(miSpiFd, SPI_IOC_RD_MODE, &bSpiMode);
	printf("[INFO] ioctl->SPI_IOC_RD_MODE = %d\n", iStatusValue);
	if(iStatusValue < 0)
	{
		//printf("Could not set SPIMode (RD)...ioctl fail");
		//exit(1);
        iStatusValue +=-3;
	}

	iStatusValue = ioctl(miSpiFd, SPI_IOC_WR_BITS_PER_WORD, &bSpiBitsPerWord);
	printf("[INFO] ioctl->SPI_IOC_WR_BITS_PER_WORD = %d\n", iStatusValue);
	if(iStatusValue < 0)
	{
		//printf("Could not set SPI bitsPerWord (WR)...ioctl fail");
		//exit(1);
        iStatusValue += -4;
	}

	iStatusValue = ioctl(miSpiFd, SPI_IOC_RD_BITS_PER_WORD, &bSpiBitsPerWord);
	printf("[INFO] ioctl->SPI_IOC_RD_BITS_PER_WORD = %d\n", iStatusValue);
	if(iStatusValue < 0)
	{
		//printf("Could not set SPI bitsPerWord(RD)...ioctl fail");
		//exit(1);
        iStatusValue += -5;
	}

	iStatusValue = ioctl(miSpiFd, SPI_IOC_WR_MAX_SPEED_HZ, &uiSpiSpeed);
	printf("[INFO] ioctl->SPI_IOC_WR_MAX_SPEED_HZ = %d\n", iStatusValue);
	if(iStatusValue < 0)
	{
		//printf("Could not set SPI speed (WR)...ioctl fail");
		//exit(1);
        iStatusValue += -6;
	}

	iStatusValue = ioctl(miSpiFd, SPI_IOC_RD_MAX_SPEED_HZ, &uiSpiSpeed);
	printf("[INFO] ioctl->SPI_IOC_RD_MAX_SPEED_HZ = %d\n", iStatusValue);
	if(iStatusValue < 0)
	{
		//printf("Could not set SPI speed (RD)...ioctl fail");
		//exit(1);
        iStatusValue += -7;
	}
	
	return(iStatusValue);
}

///////////////////////////////////////////////////
// Helper function for SPI -- SpiCloseSACPort
// Error return values:
// -1: Error - Could not close SPI device
///////////////////////////////////////////////////
int SpiCloseSACPort(void)
{
	int iStatusValue = -1;
	printf("[INFO] Closing SPI device /dev/spidev0.0 ...");
    
    if (miSpiFd > 0)
    {
        iStatusValue = close(miSpiFd);
    }
    else
    {
        iStatusValue = 100;
    }
	printf(" with exit code %d\n", iStatusValue);
	return(iStatusValue);
}
///////////////////////////////////////////////////

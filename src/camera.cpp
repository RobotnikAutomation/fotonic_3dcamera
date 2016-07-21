/*********************************************************
*
* Fotonic Confidential Information
*
* (c) Copyright Fotonic 2010. All Rights Reserved.
*
* This code is provided AS-IS; there is NO WARRANTY,
* expressed or implied.
*
* THE CONTENTS OF THIS FILE MAY NOT BE COPIED
* NOR DISTRIBUTED TO ANY OTHER PARTY, PURSUANT
* TO THE TERMS OF THE PREVAILING LICENSING AGREEMENT.
*
* THIS COPYRIGHT STATEMENT MUST NOT BE ALTERED NOR REMOVED
*
**********************************************************/

/*********************************************************
*
* Robotnik - adaption for ROS
*
**********************************************************/

// Robotnik hack
// #include <windows.h>

#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include "fotonic_3dcamera/camera.h"
#include <string.h>  // added for memset
#include <ros/ros.h>


// Robotnik hack
/*

// include camera API library files
#pragma comment (lib, "fz_api.lib")

// a way to make class thread safe
class C_Critical {
public:
	C_Critical(CRITICAL_SECTION *pCS) { m_pStatusCS = pCS; EnterCriticalSection(m_pStatusCS); }
	~C_Critical()                     { LeaveCriticalSection(m_pStatusCS); }
private:
	CRITICAL_SECTION *m_pStatusCS;
};
#define GUARD_FUNCTION(pCS) C_Critical criticalInstance(pCS);
*/

// constructor
C_FZCamera::C_FZCamera()
{
	// Robotnik hack
	//InitializeCriticalSection(&m_statusCS);
	//GUARD_FUNCTION(&m_statusCS);

	m_bInited = false;
	m_bStarted = false;
	m_iNumDevices = 0;
	// m_hDevice = NULL;
    m_hDevice = 0;
	m_pLastImage = NULL;
	m_iLastImgSize = 0;
	m_bStatEnable = false;

	m_szVersionAPI[0] = 0;
	m_szVersionCA[0] = 0;
	m_szVersionDE[0] = 0;
	m_szVersionHW[0] = 0;
	m_szSerial[0] = 0;
	m_iMeassuredFrameRate = 0;
	m_iReportedFrameRate = 0;
	m_iFrameCounter = 0;
	m_iChipTemperature = 0;
	memset(&m_stFrmHdr, 0, sizeof(m_stFrmHdr));

	m_iMode = DE_MODE_MULTI_ST;
	m_iShutterMs10 = 100;
	m_iFPS = 40;
	m_iFPSDiv = 1;

	m_iMSShutter2Ms10 = 50;
	m_iMSSaturation = 800;
	//m_iMSCompareType = DE_MS_TYPE_SATURATION;

	StatisticsWOI((160/2)-(14/2), (120/2)-(14/2), 14, 14); //default
	CalcStatistics(); //resets values

}

// destructor
C_FZCamera::~C_FZCamera()
{
	// Robotnik hack
	//GUARD_FUNCTION(&m_statusCS);
	if(m_pLastImage) delete[] m_pLastImage;
	if(!m_bInited) return;

	Stop();
	FZ_Close(m_hDevice);

	// Robotnik 
	FZ_Result result = FZ_Exit();
	ROS_INFO("C_FZCamera destructor : FZ_Exit returned %d", (int) result);
}

// Robotnik
FZ_Result C_FZCamera::Init()
{
	// 
	FZ_Result result = FZ_Init();

	return result;
}

// static method to enable/disable API logging
void C_FZCamera::SetLogging(bool bToConsole, char *szFilename)
{
	// set desired logging
	int iFlags = 0;
	if(bToConsole) iFlags |= FZ_LOG_TO_STDOUT | FZ_LOG_OPEN_CONSOLE;
	if(szFilename) iFlags |= FZ_LOG_TO_FILE;
	// change to following line to specify the wanted logging level
	iFlags |= FZ_LOG_ERROR; //FZ_LOG_ERROR | FZ_LOG_WARN | FZ_LOG_INFO | FZ_LOG_TRACE;
	//FZ_SetLogging(iFlags, szFilename);

	// set desired FZ-API logging
	FZ_SetLogging(iFlags, NULL, NULL);
}

// find accessable cameras
// always valid
int C_FZCamera::Enumerate()
{
	m_iNumDevices = MAX_DEVICES; //max
    
	// find connected sensors
	FZ_Result iResult = FZ_EnumDevices2(m_aDeviceInfo, &m_iNumDevices);
	
	//ROS_INFO("m_aDeviceInfo[1] szPath %s  szSerial %s", m_aDeviceInfo[1].szPath, m_aDeviceInfo[1].szSerial);
	
	if(iResult==FZ_TOO_MANY_DEVICES) iResult = FZ_Success;
	if( iResult!=FZ_Success || m_iNumDevices<1 ) {
		return 0;
	}
	return m_iNumDevices;
}

// return the device name for the given index
// valid after Enumerate()
const char *C_FZCamera::GetDescription(int iDevice)
{
	if(iDevice<0 || iDevice>=m_iNumDevices) return NULL;
// sprintf_s(m_szDeviceDescription, sizeof(m_szDeviceDescription), "%s [%s]", m_aDeviceInfo[iDevice].szShortName, m_aDeviceInfo[iDevice].szSerial);
	sprintf(m_szDeviceDescription, "%s [%s]", m_aDeviceInfo[iDevice].szShortName, m_aDeviceInfo[iDevice].szSerial);
	return m_szDeviceDescription;
}

// initializes the camera with the given path
// always valid
bool C_FZCamera::SetActive(char *szDevicePath)
{
	// Robotnik hack
	//GUARD_FUNCTION(&m_statusCS);

	FZ_CmdRespCode_t iRespCode;
	int iRespBytes;
	FZ_Result iResult;
	int iResultIoctl;

	// stop active camera
	if(m_bInited) {
		Stop();
		FZ_Close(m_hDevice);
		m_hDevice = 0;
		m_bInited = false;
	}
	if(!szDevicePath) return false;

	// open the given sensor
	iResult = FZ_Open(szDevicePath, 0, &m_hDevice);

	if( iResult!=FZ_Success ) {
		ROS_ERROR("C_FZCamera::SetActive - Could not open device (code 0x%x)!", iResult);
		m_hDevice = 0;
		return false;
	}

	// error checking is done from here on only once after all ioctls have been called,
	//  this works using the bitwise OR operator since FZ_Success equals 0

	// get information strings
	// NOTE: iRespBytes returned is fixed on new camera sw to include null char at the end,
	//  so the null termination is not neccesary any more, but kept to be compatible with old cameras
	iRespBytes = 127;
	iResultIoctl = FZ_IOCtl(m_hDevice,
		CMD_API_GET_VERSION, NULL, 0,
		&iRespCode, m_szVersionAPI, &iRespBytes);
	m_szVersionAPI[iRespBytes] = 0;
	iRespBytes = 127;
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_GET_VERSION, NULL, 0,
		&iRespCode, m_szVersionDE, &iRespBytes);
	m_szVersionDE[iRespBytes] = 0;
	iRespBytes = 127;
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_CA_GET_VERSION, NULL, 0,
		&iRespCode, m_szVersionCA, &iRespBytes);
	m_szVersionCA[iRespBytes] = 0;
	iRespBytes = 127;
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_GET_PCODE, NULL, 0,
		&iRespCode, m_szVersionHW, &iRespBytes);
	m_szVersionHW[iRespBytes] = 0;
	iRespBytes = 127;
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_GET_UNIT_NO, NULL, 0,
		&iRespCode, m_szSerial, &iRespBytes);
	m_szSerial[iRespBytes] = 0;

	// get woi
	short woi_top, woi_bottom, woi_left, woi_right;
	iRespBytes = sizeof(short);
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_GET_WOI_TOP, NULL, 0,
		&iRespCode, &woi_top, &iRespBytes);
	iRespBytes = sizeof(short);
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_GET_WOI_BOTTOM, NULL, 0,
		&iRespCode, &woi_bottom, &iRespBytes);
	iRespBytes = sizeof(short);
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_GET_WOI_LEFT, NULL, 0,
		&iRespCode, &woi_left, &iRespBytes);
	iRespBytes = sizeof(short);
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_GET_WOI_RIGHT, NULL, 0,
		&iRespCode, &woi_right, &iRespBytes);
	m_iCols = (woi_right - woi_left) + 1;
	m_iRows = (woi_bottom - woi_top) + 1;

	// set mode
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_SET_MODE, &m_iMode, sizeof(m_iMode),
		&iRespCode, NULL, NULL);
	// set shutter
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_SET_SHUTTER, &m_iShutterMs10	, sizeof(m_iShutterMs10),
		&iRespCode, NULL, NULL);
	// set frame rate (and divisor)
	iResultIoctl |= FZ_IOCtl(m_hDevice,
		CMD_DE_SET_FPS, &m_iFPS, sizeof(m_iFPS),
		NULL, NULL, NULL);
	FZ_IOCtl(m_hDevice, // ignore result since old cameras dont have this command
		CMD_DE_SET_FPS_DIVISOR, &m_iFPSDiv, sizeof(m_iFPSDiv),
		NULL, NULL, NULL);
	// set multi shutter values
	// ignore result since old cameras dont have this
	unsigned short iShutters[2] = {m_iShutterMs10, m_iMSShutter2Ms10};
	FZ_IOCtl(m_hDevice, CMD_DE_SET_SHUTTER, &iShutters[0], sizeof(iShutters), NULL, NULL, NULL);
	FZ_IOCtl(m_hDevice, CMD_DE_SET_MS_SATURATION, &m_iMSSaturation, sizeof(m_iMSSaturation), NULL, NULL, NULL);
	//FZ_IOCtl(m_hDevice, CMD_DE_SET_MS_TYPE, &m_iMSCompareType, sizeof(m_iMSCompareType), NULL, NULL, NULL);

	// set light switch on
//	short iLightOn = 1;
//	iResultIoctl |= FZ_IOCtl(m_hDevice,
//		CMD_DE_SET_LSSWITCH, &iLightOn, sizeof(iLightOn),
//		&iRespCode, NULL, NULL);

	if(iResultIoctl != (int)FZ_Success) {
	    ROS_ERROR("C_FZCamera::SetActive - Some error occured during the sensor open process");
		ROS_ERROR("C_FZCamera::SetActive - The camera might not work as expected.");
		//no special action is taken here due to the error
	}
	StatisticsWOI((m_iCols/2)-(14/2), (m_iRows/2)-(14/2), 14, 14); //default

	m_bInited = true;
	return true;
}

// returns current configuration X x Y 
void C_FZCamera::GetPictureRowCol(int* iCols, int* iRows)
{
	*iCols = m_iCols;
	*iRows = m_iRows;
}

// initializes the camera with the given enumerated index
// valid after Enumerate()
bool C_FZCamera::SetActive(int iDevice)
{
	char *szDevicePath = NULL;
	if(iDevice>=0 && iDevice<m_iNumDevices) {
		szDevicePath = m_aDeviceInfo[iDevice].szPath;
	}

	// open the given sensor
	return SetActive(szDevicePath);
}

// sets mode to the active camera
// always valid
bool C_FZCamera::SetMode(unsigned short iMode)
{
	FZ_CmdRespCode_t iRespCode;

	int iOldMode = m_iMode;
	m_iMode = iMode;
	if(!m_bInited) return false;
	if(iOldMode==iMode) return true;

	FZ_Result iResult = FZ_IOCtl(m_hDevice, CMD_DE_SET_MODE, &m_iMode, sizeof(m_iMode), &iRespCode, NULL, NULL);

	SetFrameRate(0xffff, 0xffff); //force setting the same

	return (iResult==FZ_Success);
}

// sets shutter to the active camera
// always valid
bool C_FZCamera::SetShutter(unsigned short iShutterMs10)
{
	FZ_CmdRespCode_t iRespCode;

	int iOldShutterMs10 = m_iShutterMs10;
	m_iShutterMs10 = iShutterMs10;
	if(!m_bInited) return false;
	if(iOldShutterMs10==iShutterMs10) return true;

	FZ_Result iResult = FZ_IOCtl(m_hDevice, CMD_DE_SET_SHUTTER, &m_iShutterMs10, sizeof(m_iShutterMs10), &iRespCode, NULL, NULL);

	SetFrameRate(0xffff, 0xffff); //force setting the same

	return (iResult==FZ_Success);
}

// sets frame rate to the active camera
// always valid
bool C_FZCamera::SetFrameRate(unsigned short iFPS, unsigned short iFPSDiv)
{
	FZ_CmdRespCode_t iRespCode;

	int iOldFPS = m_iFPS;
	int iOldFPSDiv = m_iFPSDiv;
	if(iFPS!=0xffff) m_iFPS = iFPS;
	if(iFPSDiv!=0xffff) m_iFPSDiv = iFPSDiv;
	if(!m_bInited) return false;

	FZ_Result iResult = FZ_Success;
	if(iOldFPS!=iFPS) iResult = FZ_IOCtl(m_hDevice, CMD_DE_SET_FPS, &m_iFPS, sizeof(m_iFPS), &iRespCode, NULL, NULL);
	//set divisor but ignore result since old cameras dont have this command
	if(iOldFPSDiv!=iFPSDiv) FZ_IOCtl(m_hDevice, CMD_DE_SET_FPS_DIVISOR, &m_iFPSDiv, sizeof(m_iFPSDiv), &iRespCode, NULL, NULL);

	return (iResult==FZ_Success);
}


// multi shutter control
// always valid
bool C_FZCamera::SetMSShutter2(unsigned short iShutterMs10)
{
	FZ_CmdRespCode_t iRespCode;

	int iOldShutterMs10 = m_iMSShutter2Ms10;
	m_iMSShutter2Ms10 = iShutterMs10;
	if(!m_bInited) return false;
	if(iOldShutterMs10==iShutterMs10) return true;

	FZ_SHUTTER_EXT stShutters;
	stShutters.num_shutters = 1;
	stShutters.shutters[0].shutter_nr = 2;
	stShutters.shutters[0].shutter_time = m_iMSShutter2Ms10;

	FZ_Result iResult = FZ_IOCtl(m_hDevice, CMD_DE_SET_SHUTTER_EXT, &stShutters, sizeof(stShutters), &iRespCode, NULL, NULL);
	return (iResult==FZ_Success);
}
bool C_FZCamera::SetMSSaturation(unsigned short iSaturation)
{
	FZ_CmdRespCode_t iRespCode;

	m_iMSSaturation = iSaturation;
	if(!m_bInited) return false;

	FZ_Result iResult = FZ_IOCtl(m_hDevice, CMD_DE_SET_MS_SATURATION, &m_iMSSaturation, sizeof(m_iMSSaturation), &iRespCode, NULL, NULL);
	return (iResult==FZ_Success);
}

/*
bool C_FZCamera::SetMSCompareType(unsigned short iType)
{
	FZ_CmdRespCode_t iRespCode;

	m_iMSCompareType = iType;
	if(!m_bInited) return false;

	FZ_Result iResult = FZ_IOCtl(m_hDevice, CMD_DE_SET_MS_TYPE, &m_iMSCompareType, sizeof(m_iMSCompareType), &iRespCode, NULL, NULL);
	return (iResult==FZ_Success);
}
*/

bool C_FZCamera::SetFiltering(unsigned short iLerpOn, unsigned short iEdgeOn, unsigned short iEdgeMinB,
	unsigned short iEdgeDiff1, unsigned short iEdgeDiff2, unsigned short iEdgeDiff3)
{
	FZ_CmdRespCode_t iRespCode;

	m_iLerpOn = iLerpOn;
	m_iEdgeParams[0] = iEdgeOn;
	m_iEdgeParams[1] = iEdgeMinB;
	m_iEdgeParams[2] = iEdgeDiff1;
	m_iEdgeParams[3] = iEdgeDiff2;
	m_iEdgeParams[4] = iEdgeDiff3;
	if(!m_bInited) return false;

	int iResult;

	iResult = FZ_IOCtl(m_hDevice, CMD_DE_SET_LERP_FILTER, &m_iLerpOn, sizeof(m_iLerpOn), &iRespCode, NULL, NULL);
	iResult |= FZ_IOCtl(m_hDevice, CMD_DE_SET_EDGE_FILTER, &m_iEdgeParams[0], sizeof(m_iEdgeParams), &iRespCode, NULL, NULL);
	return (iResult==FZ_Success);

        return true;
}

// start the active camera
// valid after a successful SetActive()
bool C_FZCamera::Start()
{
	if(!m_bInited) return false;

	FZ_CmdRespCode_t resp;
	if( FZ_IOCtl(m_hDevice, CMD_DE_SENSOR_START, NULL, 0, &resp, NULL, NULL) != FZ_Success ) {
        ROS_ERROR("C_FZCamera::Start  - Start sensor failed");
		return false;
	}

	m_bStarted = true;
	return true;
}

// stop the active camera
// valid after a successful Start()
void C_FZCamera::Stop()
{
	if(!m_bInited) return;
	m_bStarted = false;

	FZ_CmdRespCode_t resp;
	FZ_IOCtl(m_hDevice, CMD_DE_SENSOR_STOP, NULL, 0, &resp, NULL, NULL);
}

/*
// gets next frame from a started camera
// valid after a successful Start()
short *C_FZCamera::GetFrame()
{
    // Robotnik hack
    // GUARD_FUNCTION(&m_statusCS);
	if(!m_bInited) return NULL;

	//(re)allocate memory if neccesary
 	int iCurrImgSize = m_iCols*m_iRows*4; //BZXY (8 bytes per pixel)
	if(iCurrImgSize!=m_iLastImgSize) {
		if(m_pLastImage) delete[] m_pLastImage;
		m_pLastImage = new short[iCurrImgSize];
		m_iLastImgSize = iCurrImgSize;
	}

	size_t iBufSize = m_iCols*m_iRows*8;
	FZ_Result iResult = FZ_GetFrame(m_hDevice, &m_stFrmHdr, m_pLastImage, &iBufSize);
	if( iResult!=FZ_Success ) {
		return NULL;
	}

	m_iMeassuredFrameRate = m_stFrmHdr.measuredframerate;
	m_iReportedFrameRate = m_stFrmHdr.reportedframerate;
	m_iFrameCounter = m_stFrmHdr.framecounter;
	m_iChipTemperature = m_stFrmHdr.temperature; //celcius *10

	CalcStatistics();

	return m_pLastImage;
}
*/

// gets next frame from a started camera
// valid after a successful Start()
short *C_FZCamera::GetFrame()
{
	// GUARD_FUNCTION(&m_statusCS);
	if(!m_bInited) return NULL;

	//(re)allocate memory if neccesary
// 	int iMaxImgSize = 640*480*4; //max 640*480 BZXY (8 bytes per pixel)
 	int iMaxImgSize = 160*120*4; //max 640*480 BZXY (8 bytes per pixel)
	if(iMaxImgSize!=m_iLastImgSize) {
		if(m_pLastImage) delete[] m_pLastImage;
		m_pLastImage = new short[iMaxImgSize];
	}

	size_t iBufSize = iMaxImgSize*2;
	FZ_Result iResult = FZ_GetFrame(m_hDevice, &m_stFrmHdr, m_pLastImage, &iBufSize);
	if( iResult!=FZ_Success ) {
		return NULL;
	}
	m_iLastImgSize = (int)iBufSize;

	m_iMeassuredFrameRate = m_stFrmHdr.measuredframerate;
	m_iReportedFrameRate = m_stFrmHdr.reportedframerate;
	m_iFrameCounter = m_stFrmHdr.framecounter;
	m_iChipTemperature = m_stFrmHdr.temperature; //celcius *10

	m_iCols = m_stFrmHdr.ncols;
	m_iRows = m_stFrmHdr.nrows;
	StatisticsWOI((m_iCols/2)-(14/2), (m_iRows/2)-(14/2), 14, 14); //default
	CalcStatistics();

/*
	// get cpu temp
	if(m_iFrameCounter%300==1) {
		FZ_CmdRespCode_t iRespCode;
		int iRespBytes = sizeof(short);
		FZ_Result iResultIoctl = FZ_IOCtl(m_hDevice,
			CMD_DE_GET_CPU_TEMP, NULL, 0,
			&iRespCode, &m_iCpuTemperature, &iRespBytes);
		if(iRespCode != (int)R_CMD_DE_ACK) m_iCpuTemperature = 0;
	}
*/

	return m_pLastImage;
}


// gets frame header for the last frame
// valid after a successful GetFrame()
FZ_FRAME_HEADER *C_FZCamera::GetFrameHeader()
{
	return &m_stFrmHdr;
}

// gets information on the activa camera
// version info valid after a successful SetActive(), dynamic info valid after a successful GetFrame()
char *C_FZCamera::GetInfoString(InfoType iType, char *szOut, size_t iBufferSize)
{
	if(!iBufferSize) return NULL;

	szOut[0] = 0;
	switch(iType) {
		case API_VERSION: strcpy(szOut, m_szVersionAPI); break;
		case DE_VERSION: strcpy(szOut, m_szVersionDE); break;
		case CA_VERSION: strcpy(szOut, m_szVersionCA); break;
		case CAMERA_REV: strcpy(szOut, m_szVersionHW); break;
		case CAMERA_SERIAL: strcpy(szOut, m_szSerial); break;
		case MEASSURED_FRAMERATE:
			sprintf(szOut, "%d", m_iMeassuredFrameRate);
			break;
		case REPORTED_FRAMERATE:
			//sprintf_s(szOut, iBufferSize, "%d", m_iReportedFrameRate);
			sprintf(szOut, "%d", m_iReportedFrameRate);
			break;
		case FRAMECOUNTER:
			sprintf(szOut, "%d", m_iFrameCounter);
			break;
		case CHIP_TEMPERATURE:
			sprintf(szOut, "%.1f", (float)m_iChipTemperature/10.0f);
			break;
	}

	return szOut;
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// set the window of interest for statistics calculation
// always valid
bool C_FZCamera::StatisticsWOI(int iX, int iY, int iWidth, int iHeight)
{
    // Robotnik hack
    // GUARD_FUNCTION(&m_statusCS);

	if(iX<0 || iY<0 || iWidth<1 || iHeight<1) return false;
	m_iStatX = iX; m_iStatY = iY;
	m_iStatWidth = iWidth; m_iStatHeight = iHeight;
	return true;
}

// enable statistics calculation
// always valid
void C_FZCamera::StatisticsEnable(bool bEnable)
{
	m_bStatEnable = bEnable;
}

// gets statistics for the last frame
// valid after a successful GetFrame(), and StatisticsEnable(true)
bool C_FZCamera::GetStatistics(double *dZMean, double *dZStd, double *dBMean, double *dBStd)
{
	//if(!m_bInited) return false;
	//if(!m_bStatEnable) return false;

	if(dZMean) *dZMean = m_dZMean;
	if(dZStd)  *dZStd  = m_dZStd;
	if(dBMean) *dBMean = m_dBMean;
	if(dBStd)  *dBStd  = m_dBStd;
	return true;
}

// statistics calculation
// private, runs once per frame
void C_FZCamera::CalcStatistics()
{
	m_dZMean = m_dBMean = 0.0;
	m_dZStd = m_dBStd = 0.0;

	if(!m_bInited) return;
	if(!m_bStatEnable) return;

	//limit area
	if(m_iStatX+m_iStatWidth>m_iCols) {
		if(m_iStatWidth>m_iCols) m_iStatWidth = m_iCols;
		m_iStatX = m_iCols-m_iStatWidth;
	}
	if(m_iStatY+m_iStatHeight>m_iRows) {
		if(m_iStatHeight>m_iRows) m_iStatHeight = m_iRows;
		m_iStatY = m_iRows-m_iStatHeight;
	}

	// mean
	int iCount = m_iStatWidth*m_iStatHeight;
	int x, y;
	for(y=m_iStatY; y<m_iStatY+m_iStatHeight; y++) {
		int iRowPosB = (y*m_iCols)*4+(m_iCols*0);
		int iRowPosZ = (y*m_iCols)*4+(m_iCols*1);
		for(x=m_iStatX; x<m_iStatX+m_iStatWidth; x++) {
			//row format: Bx160, Zx160, Xx160, Yx160 (16bit per component)
			unsigned short iB = *(m_pLastImage+iRowPosB+x);
			unsigned short iZ = *(m_pLastImage+iRowPosZ+x);
			m_dZMean += iZ;
			m_dBMean += iB;
		}
	}
	m_dZMean /= iCount;
	m_dBMean /= iCount;

	// standard deviation
	for(y=m_iStatY; y<m_iStatY+m_iStatHeight; y++) {
		int iRowPosB = (y*m_iCols)*4+(m_iCols*0);
		int iRowPosZ = (y*m_iCols)*4+(m_iCols*1);
		for(x=m_iStatX; x<m_iStatX+m_iStatWidth; x++) {
			//row format: Bx160, Zx160, Xx160, Yx160 (16bit per component)
			unsigned short iB = *(m_pLastImage+iRowPosB+x);
			unsigned short iZ = *(m_pLastImage+iRowPosZ+x);
			m_dZStd += (iZ-m_dZMean) * (iZ-m_dZMean);
			m_dBStd += (iB-m_dBMean) * (iB-m_dBMean);
		}
	}
	m_dZStd = sqrt(m_dZStd/iCount);
	m_dBStd = sqrt(m_dBStd/iCount);
}

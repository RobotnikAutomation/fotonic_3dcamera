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

#ifndef __CAMERA_H__
#define __CAMERA_H__

// include camera API headers
#include "fz_api.h"

#define MAX_DEVICES 45
#define NUM_MODES_JAGUAR  13

// enum for info strings
typedef enum {
	API_VERSION,
	DE_VERSION,
	CA_VERSION,
	CAMERA_REV,
	CAMERA_SERIAL,
	MEASSURED_FRAMERATE,
	REPORTED_FRAMERATE,
	FRAMECOUNTER,
	CHIP_TEMPERATURE,
} InfoType;

// camera handling class
class C_FZCamera
{
public:
	C_FZCamera();
	~C_FZCamera();

	static void SetLogging(bool bToConsole, char *szFilename);
	
	// void FZLogCallback(char *szMetadata, char *szMessage);

	// camera initialization
	FZ_Result Init();  // Robotnik
	int Enumerate();
	const char *GetDescription(int iDevice);
	bool SetActive(int iDevice);
	bool SetActive(char *szDevicePath);
	bool IsStarted() {return m_bStarted;};

	// camera control
	bool Start();
	void Stop();
	bool SetMode(unsigned short iMode);
	bool SetShutter(unsigned short iShutterMs10); // in 0.1 ms
	bool SetFrameRate(unsigned short iFPS, unsigned short iFPSDiv);
	//bool SetWOI(int iX, int iY, int iWidth, int iHeight); // not supported by camera yet
	bool SetMSShutter2(unsigned short iShutterMs10); // in 0.1 ms
	bool SetMSSaturation(unsigned short iSaturation);
	bool SetMSCompareType(unsigned short iType);
	bool SetFiltering(unsigned short iLerpOn, unsigned short iEdgeOn, unsigned short iEdgeMinB,
		unsigned short iEdgeDiff1, unsigned short iEdgeDiff2, unsigned short iEdgeDiff3);


	// image retreaval
	short *GetFrame(); // blocks until a frame can be read if no frame is available
	FZ_FRAME_HEADER *GetFrameHeader();

	// general info
	char *GetInfoString(InfoType iType, char *szOut, size_t iBufferSize);

	// image statistics
	bool StatisticsWOI(int iX, int iY, int iWidth, int iHeight);
	void StatisticsEnable(bool bEnable);
	bool GetStatistics(double *dZMean, double *dZStd, double *dBMean, double *dBStd);

	// get 3d image configuration 
	// returns current configuration X x Y 
	void GetPictureRowCol(int* iCols, int* iRows);

private:
	// state variables
	bool m_bInited;
	bool m_bStarted;

	// FZ enumeration variables
	int m_iNumDevices;
	FZ_DEVICE_INFO m_aDeviceInfo[MAX_DEVICES];
	char m_szDeviceDescription[96];

	// FZ settings variables
	unsigned short m_iMSCompareType;
	unsigned short m_iMode;
	unsigned short m_iFPS, m_iFPSDiv;
	unsigned short m_iShutterMs10;

	unsigned short m_iMSShutter2Ms10;
	unsigned short m_iMSSaturation;

	unsigned short m_iLerpOn;
	unsigned short m_iEdgeParams[5];


	// other FZ variables
	FZ_Device_Handle_t m_hDevice;
	FZ_FRAME_HEADER m_stFrmHdr;

	// last image retreived
	int m_iCols, m_iRows;
	short *m_pLastImage;
	int m_iLastImgSize;

	// info
	char m_szVersionAPI[128];
	char m_szVersionCA[128];
	char m_szVersionDE[128];
	char m_szVersionHW[128];
	char m_szSerial[128];
	int m_iMeassuredFrameRate;
	int m_iReportedFrameRate;
	int m_iChipTemperature;
	int m_iFrameCounter;

	// statistics
	void CalcStatistics();
	int m_iStatX, m_iStatY, m_iStatWidth, m_iStatHeight;
	bool m_bStatEnable;
	double m_dZMean, m_dBMean;
	double m_dZStd, m_dBStd;

	// Robotnik hack
	// thread safety
	// CRITICAL_SECTION m_statusCS;
};


#endif

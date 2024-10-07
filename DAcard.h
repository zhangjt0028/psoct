#include "Usb3020.h"
#include <fstream>
#include <cmath>
#include <windows.h>
#include <string>

class DA_USB3020
{
public:
	const static short Voltage_0V = 32768; 
    const static short Voltage_5V = 65535; 
    const static short Voltage_5V_neg = 0;
	const static int AVERAGE_NUM = 3;	

    DA_USB3020();
	void CalculateDAdata();
	bool InitDAForScan();
	bool EnableDA();
	bool DisableDA();
	~DA_USB3020();

protected:

private:

	PUSB3020_SEGMENT_INFO	pSegmentInfoX;
	PUSB3020_SEGMENT_INFO	pSegmentInfoY;
	PUSB3020_SEGMENT_INFO	pSegmentInfoT;


	unsigned short* pDataX;
	unsigned short* pDataT;
	unsigned short* pDataY;

	HANDLE hDevice;					

	unsigned int BScanlines;
	unsigned int BScanPerSec;
	unsigned long SamplesPerSec; 
	unsigned long ZeroBufferPoint;
    
	int xscanMode;
    int yscanMode;

	unsigned long len_Transit;
	unsigned long len_Total;
	unsigned long len_Scan;

	short LinearCut(unsigned int start, unsigned int end, unsigned long length, unsigned int i);
};

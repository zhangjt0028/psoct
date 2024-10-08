#include "Usb3020.h"
#include <fstream>
#include <cmath>
#include <windows.h>
#include <string>

class DA_USB3020
{
public:
	const static int Voltage_0V = 32768; 
    const static int Voltage_5V = 65535; 
    const static int Voltage_5V_neg = 0;
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

	int BScanlines;
	int BScanPerSec;
	int SamplesPerSec; 
	int ZeroBufferPoint;

	int len_Transit;
	int len_Total;
	int len_Scan;

	int LinearCut(int start, int end, int length, int i);
};

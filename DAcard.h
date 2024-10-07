#include "Usb3020.h"
#include <fstream>
#include <cmath>
#include <windows.h>
#include <string>

//#include "mainwidget.h"

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

class DA_USB3020
{
public:
	const static short Voltage_0V = 32768; // 0V �͵�ƽ
    const static short Voltage_5V = 65535; // 5V �ߵ�ƽ
    const static short Voltage_5V_neg = 0;
	const static int AVERAGE_NUM = 3;	// Ѫ���㷨ƽ������

	// 15us ����ʱ�䣬2D crossɨ��ʱ�м�����������150��
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

	HANDLE hDevice;					// �豸������

	unsigned int BScanlines;
	unsigned int BsacnPerSec;
	unsigned long SamplesPerSec; // 1M������
	unsigned long ZeroBufferPoint; // 0V��ѹ�ĵ���������ʱ��0.1ms
    
	int xscanMode;
    int yscanMode;

	unsigned long len_Transit;
	unsigned long len_Total;
	unsigned long len_Scan;

	short LinearCut(unsigned int start, unsigned int end, unsigned long length, unsigned int i);
};

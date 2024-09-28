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
	
	const static int MODE_2D_SCAN_REPEAT = 2;
	const static int MODE_3D_SCAN_REPEAT = 3;

	const static int STATUS_RUNNING = 1;
	const static int STATUS_STOP = 0;

	const static int AVERAGE_NUM = 4;	// Ѫ���㷨ƽ������

	// 15us ����ʱ�䣬2D crossɨ��ʱ�м�����������150��
    DA_USB3020();
	void CalculateDAdata();
	bool InitDAForScan(int mode);
	bool EnableDA();
	bool DisableDA();
	bool StopScan();
	~DA_USB3020();

protected:

private:
	int curMode;
	int curStatus;

	PUSB3020_SEGMENT_INFO	pSegmentInfoX;
	PUSB3020_SEGMENT_INFO	pSegmentInfoY;
	PUSB3020_SEGMENT_INFO	pSegmentInfoT;

	HANDLE hDevice;					// �豸������
	USB3020_STATUS_DA m_DAStatus;	// DA״̬

	short LinearCut(unsigned int start, unsigned int end, unsigned long length, unsigned int i);
	unsigned short* pDataX;
	unsigned short* pDataT;
	unsigned short* pDataY;

	unsigned int Voltage;
	double Frequency;
	double duty_cycle;
	unsigned int BScanlines;

	unsigned long SamplesPerSec; // 1M������
	unsigned long ZeroBufferPoint; // 0V��ѹ�ĵ���������ʱ��0.1ms
	unsigned long len_Transit;
    int xscanMode;
    int yscanMode;

	unsigned long len_Total;
	unsigned long len_Scan;
};

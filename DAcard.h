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
    //const static short Voltage_5V = 54395; // 3.3V �ߵ�ƽ
	const static int MODE_2D_CROSS_SCAN = 2;
	const static int MODE_1D_SCAN = 1;
	const static int MODE_3D_SCAN = 3;
	const static int MODE_2D_SCAN_REPEAT = 4;
	const static int MODE_3D_SCAN_REPEAT = 5;

	const static int STATUS_RUNNING = 1;
	const static int STATUS_STOP = 0;

	const static int AVERAGE_NUM = 1;	// Ѫ���㷨ƽ������

	// 15us ����ʱ�䣬2D crossɨ��ʱ�м�����������150��
    DA_USB3020(unsigned long SamplesPerSec = 200319L, unsigned long ZeroBufferPoint = 10,
        unsigned long CosTransitPoint = 100, int xscanMode = 0, int yscanMode = 2);

	// ����ÿ��ͨ����DA����
	void CalculateDAdata(unsigned int Voltage, double Frequency, 
        double duty_cycle, unsigned int BScanlines,int xscanMode, int yscanMode);

	// ���㳤�ȣ�������CalculateDAdata����һ�£���InitDA֮ǰ�������
	// CalculateLength����CalculateDAdata����
	void CalculateLength(unsigned int Voltage, double Frequency, 
		double duty_cycle, unsigned int BScanlines);

	// ����DA�������ʼ��ʱ����ã������ʼ��ʧ�ܿ��ٴε���
	bool ConnectDA();

	// ������д�뵽DA��
    bool WriteDataToDA();

	// StartScan�������Զ����ô˺���������ʹ�ò���ֱ�ӵ��ô˺���
	bool InitDAForScan(int mode, bool thenEnableDA = true);

	// ʹ��DAͨ����InitDAForScan���Զ����ô˺���������ʹ�ò���ֱ�ӵ��ô˺���
	bool EnableDA();

	// ֹͣDA������ʱ�������
	bool DisableDA();

	// �ͷ�DA������ʱ�������
	bool ReleaseDA();

	// �Ƿ�ɹ�����DA
	bool isConnected();

	// �Ƿ��Ѿ���ʼ���ɹ�����������ɼ��ź�
	bool isReadyForScan();

	// ��ʼ1ά�ɼ�
	bool Start1Dscan();

	// ��ʼ2ά�ɼ�
	bool Start2Dscan();

	// ��ʼ�ظ�2ά�ɼ�
	bool Start2DscanRepeat();

	// ��ʼ3ά�ɼ�
	bool Start3Dscan();

	// ��ʼ�ظ�3ά�ɼ�
	bool Start3DscanRepeat();

	// ֹͣ�ɼ�
	bool StopScan();

	// ��ȡDA״̬
	void GetDAstatus(int channel, long& bTrigFlag, long& bConverting, 
		long& nCurSegNum, long& nCurSegAddr, long& nCurLoopCount, long& nCurSegLoopCount);

	// ��������
	~DA_USB3020();

protected:

private:
	int curMode;
	int curStatus;
	PUSB3020_SEGMENT_INFO	pSegmentInfoX;

	PUSB3020_SEGMENT_INFO	pSegmentInfoY;

	PUSB3020_SEGMENT_INFO	pSegmentInfoT;

	PUSB3020_PARA_DA		pParaDA_2D1D;
	PUSB3020_PARA_DA		pParaDA_3D;
	PUSB3020_PARA_DA		pParaDA_2D_Repeat;
	PUSB3020_PARA_DA		pParaDA_3D_Repeat;

	HANDLE hDevice;					// �豸������
	BOOL bStatus;					// �����ķ���ֵ
	USB3020_PARA_DA DAPara;			// DA�����ṹ��
	LONG SegmentCount;				// �ֶ�����
	USB3020_STATUS_DA m_DAStatus;	// DA״̬

	void GenDataX();
	void GenDataY();
	void GenDataT();

	unsigned short* pDataX;
	unsigned short* pDataT;
	unsigned short* pDataY;

	unsigned int Voltage;
	double Frequency;
	double duty_cycle;
	unsigned int BScanlines;

	unsigned long SamplesPerSec; // 1M������
	unsigned long ZeroBufferPoint; // 0V��ѹ�ĵ���������ʱ��0.1ms
	unsigned long CosTransitPoint; // 500Hz��ƫתƵ�ʣ��Ӹߵ�ѹ���͵�ѹ
    int xscanMode;
    int yscanMode;

	unsigned long len_RampScan;
	unsigned long len_LinearScan;

	unsigned long x_len;
	unsigned long x_T1_beg;
	unsigned long x_T1_len;
	unsigned long x_T1_1_beg;
	unsigned long x_T1_1_len;
	unsigned long x_T1_2_beg;
	long x_T1_2_len;
	unsigned long x_T1_3_beg;
	unsigned long x_T1_3_len;
	unsigned long x_T1_4_beg;
	unsigned long x_T1_4_len;
	unsigned long x_T1_5_beg;
	unsigned long x_T1_5_len;

	unsigned long x_T2_beg;
	unsigned long x_T2_len;

	unsigned long x_T3_beg;
	unsigned long x_T3_len;
	unsigned long x_T3_1_beg;
	unsigned long x_T3_1_len;
	unsigned long x_T3_2_beg;
	long x_T3_2_len;
	unsigned long x_T3_3_beg;
	unsigned long x_T3_3_len;

	unsigned long x_T4_beg;
	unsigned long x_T4_len;

	unsigned long y_len;
	unsigned long y_T1_beg;
	unsigned long y_T1_len;
	unsigned long y_T1_1_beg;
	unsigned long y_T1_1_len;
	unsigned long y_T1_2_beg;
	unsigned long y_T1_2_len;
	unsigned long y_T1_3_beg;
	unsigned long y_T1_3_len;
	unsigned long y_T1_4_beg;
	long y_T1_4_len;
	unsigned long y_T1_5_beg;
	unsigned long y_T1_5_len;

	unsigned long y_T2_beg;
	unsigned long y_T2_len;

	unsigned long y_T3_beg;
	unsigned long y_T3_len;

	unsigned long y_TBscans_beg;
	unsigned long y_TBScans_len;

	unsigned long y_T4_beg;
	unsigned long y_T4_len;

	unsigned long y_T5_beg;
	unsigned long y_T5_len;

};

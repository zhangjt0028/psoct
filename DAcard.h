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
	const static short Voltage_0V = 32768; // 0V 低电平
    const static short Voltage_5V = 65535; // 5V 高电平
    //const static short Voltage_5V = 54395; // 3.3V 高电平
	const static int MODE_2D_CROSS_SCAN = 2;
	const static int MODE_1D_SCAN = 1;
	const static int MODE_3D_SCAN = 3;
	const static int MODE_2D_SCAN_REPEAT = 4;
	const static int MODE_3D_SCAN_REPEAT = 5;

	const static int STATUS_RUNNING = 1;
	const static int STATUS_STOP = 0;

	const static int AVERAGE_NUM = 1;	// 血管算法平均次数

	// 15us 积分时间，2D cross扫描时中间空余的线数是150线
    DA_USB3020(unsigned long SamplesPerSec = 200319L, unsigned long ZeroBufferPoint = 10,
        unsigned long CosTransitPoint = 100, int xscanMode = 0, int yscanMode = 2);

	// 计算每个通道的DA数据
	void CalculateDAdata(unsigned int Voltage, double Frequency, 
        double duty_cycle, unsigned int BScanlines,int xscanMode, int yscanMode);

	// 计算长度，参数和CalculateDAdata必须一致，在InitDA之前必须调用
	// CalculateLength或者CalculateDAdata函数
	void CalculateLength(unsigned int Voltage, double Frequency, 
		double duty_cycle, unsigned int BScanlines);

	// 连接DA卡，类初始化时会调用，如果初始化失败可再次调用
	bool ConnectDA();

	// 把数据写入到DA中
    bool WriteDataToDA();

	// StartScan函数会自动调用此函数，正常使用不必直接调用此函数
	bool InitDAForScan(int mode, bool thenEnableDA = true);

	// 使能DA通道，InitDAForScan会自动调用此函数，正常使用不必直接调用此函数
	bool EnableDA();

	// 停止DA，正常时无需调用
	bool DisableDA();

	// 释放DA，正常时无需调用
	bool ReleaseDA();

	// 是否成功连接DA
	bool isConnected();

	// 是否已经初始化成功，可以输出采集信号
	bool isReadyForScan();

	// 开始1维采集
	bool Start1Dscan();

	// 开始2维采集
	bool Start2Dscan();

	// 开始重复2维采集
	bool Start2DscanRepeat();

	// 开始3维采集
	bool Start3Dscan();

	// 开始重复3维采集
	bool Start3DscanRepeat();

	// 停止采集
	bool StopScan();

	// 获取DA状态
	void GetDAstatus(int channel, long& bTrigFlag, long& bConverting, 
		long& nCurSegNum, long& nCurSegAddr, long& nCurLoopCount, long& nCurSegLoopCount);

	// 析构函数
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

	HANDLE hDevice;					// 设备对象句柄
	BOOL bStatus;					// 函数的返回值
	USB3020_PARA_DA DAPara;			// DA参数结构体
	LONG SegmentCount;				// 分段总数
	USB3020_STATUS_DA m_DAStatus;	// DA状态

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

	unsigned long SamplesPerSec; // 1M采样率
	unsigned long ZeroBufferPoint; // 0V电压的点数，持续时间0.1ms
	unsigned long CosTransitPoint; // 500Hz的偏转频率，从高电压到低电压
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

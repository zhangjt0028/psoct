// 本程序演示了DA输出过程

#include "stdafx.h"
#include "windows.h"
#include "stdio.h"
#include "conio.h"
#include "math.h"
#include "stdlib.h"

#include "Wave.h"
#include "USB3020.h" // 驱动程序接口头件(必须)
#define ESC 27

#define BUFFER_SIZE 1024*1024*2


int main(int argc, char* argv[])
{
	HANDLE hDevice;  //  设备对象句柄
	int DeviceID;    //  设备号
	BOOL bStatus; // 函数的返回值
	LONG nWriteOffsetWords=0; // 相对于物理缓冲区(即RAM)0位置的偏移点(双点)
	LONG nWriteSizeWords;   // 读入的数据长度(双字)
	LONG nRetSizeWords;
	LONG SegmentCount = 1;
	BOOL bResetDA = FALSE; // 每次开始时，是否将RAM扫描指针复位至0段0地址
	int WaveType = 0;
	float fTrigVolt = 1000.0; // 触发电压为1000毫伏
	int SegmentIdx = 0;
	int Index = 0;
	USB3020_PARA_DA DAPara; // DA参数结构体
	USB3020_SEGMENT_INFO SegmentInfo[32];
	USB3020_STATUS_DA DAStatus; // DA状态结构体
	int SegmentAddr; // 记录段地址
	PUSHORT DABuffer = new USHORT[BUFFER_SIZE];
	PUSHORT TDABuffer = new USHORT[BUFFER_SIZE];
	int nDAChannel = 0, SegmentID = 0;
	DeviceID = 0;
	hDevice = USB3020_CreateDevice( DeviceID ); // 创建设备对象
	if(hDevice == INVALID_HANDLE_VALUE) 
	{
		printf("Create Device Error...\n");
		_getch();
		return 0;
	}

	// 将参数结构体初始化为确定值0
	memset(&DAPara, 0, sizeof(DAPara));
	DAPara.OutputRange = USB3020_OUTPUT_N10000_P10000mV;
	DAPara.Frequency = 100000; // 采集频率Hz
	DAPara.LoopCount = 0; // 整过RAM的大循环次数,=0:无限循环, =n:表示n次循环(n<2048)
	DAPara.TriggerSource = USB3020_TRIGSRC_SOFT_DA;
	DAPara.TriggerMode = USB3020_TRIGMODE_CONTINUOUS;		// 内/外触发(即AD启动)方式选择
	DAPara.TriggerDir = USB3020_TRIGDIR_POSITIVE;	// 正向/负向触发选择
	DAPara.ClockSource = USB3020_CLOCKSRC_IN;		// 时钟源选择

	// 将参数结构体初始化为确定值0
	memset(&SegmentInfo, 0, sizeof(SegmentInfo));
	SegmentCount = 5;
	SegmentInfo[0].SegLoopCount = 25;
	SegmentInfo[0].SegmentSize = 6400;

	SegmentInfo[1].SegLoopCount = 25;
	SegmentInfo[1].SegmentSize = 6400;

	SegmentInfo[2].SegLoopCount = 256;
	SegmentInfo[2].SegmentSize = 6400;

	SegmentInfo[3].SegLoopCount = 5;
	SegmentInfo[3].SegmentSize = 4096;

	SegmentInfo[4].SegLoopCount = 5;
	SegmentInfo[4].SegmentSize = 4096;
	
	SegmentInfo[5].SegLoopCount = 5;
	SegmentInfo[5].SegmentSize = 4096;

	printf("Please Input DAChannel(0-3):");
	scanf_s("%d", &nDAChannel);	
	
	bStatus = USB3020_InitDeviceDA(hDevice, SegmentCount, SegmentInfo, &DAPara, nDAChannel);
	if(!bStatus) { printf("InitDeviceDA Error...\n"); _getch(); goto ExitOut; }

	// 求得段0在DABuffer缓冲区中的地址
	SegmentAddr = 0;
	GenerateWave(&DABuffer[0], SegmentInfo[0].SegmentSize, 640, WAVE_PULSE); // 根据0段地址和长度生成正弦波数据

	// 求得段1在DABuffer缓冲区中的地址
	SegmentAddr = SegmentInfo[0].SegmentSize;
	GenerateWave(&DABuffer[SegmentAddr], SegmentInfo[1].SegmentSize, 640, WAVE_PULSE);

	// 求得段2在DABuffer缓冲区中的地址
	SegmentAddr = SegmentInfo[0].SegmentSize + SegmentInfo[1].SegmentSize;
	GenerateWave(&DABuffer[SegmentAddr], SegmentInfo[2].SegmentSize, 64, WAVE_PULSE);

	// 求得段3在DABuffer缓冲区中的地址
	SegmentAddr = SegmentInfo[0].SegmentSize + SegmentInfo[1].SegmentSize + SegmentInfo[2].SegmentSize;
	GenerateWave(&DABuffer[SegmentAddr], SegmentInfo[3].SegmentSize, 64, WAVE_PULSE);

	// 求得段4在DABuffer缓冲区中的地址
	SegmentAddr = SegmentInfo[0].SegmentSize + SegmentInfo[1].SegmentSize + SegmentInfo[2].SegmentSize + SegmentInfo[3].SegmentSize;
	GenerateWave(&DABuffer[SegmentAddr], SegmentInfo[4].SegmentSize, 64, WAVE_PULSE);

	// nWriteSizeWords = SegmentInfo[0].SegmentSize + SegmentInfo[1].SegmentSize + SegmentInfo[2].SegmentSize + SegmentInfo[3].SegmentSize;
	printf("It is writting data to RAM....\n");

	nWriteSizeWords = 0;
	for(SegmentIdx=0; SegmentIdx<SegmentCount; SegmentIdx++)
	{
		nWriteSizeWords = nWriteSizeWords + SegmentInfo[SegmentIdx].SegmentSize;
	}
	
	bStatus = USB3020_WriteDeviceBulkDA(hDevice, DABuffer, nWriteSizeWords, &nRetSizeWords, nDAChannel);
	if(!bStatus) { printf("WriteDeviceProDA Error...\n"); _getch(); goto ExitOut; }

	printf("Writing DA is over, press any key to start device....\n");
	_getch();		
	if(!USB3020_EnableDeviceDA(hDevice, nDAChannel))
	{
		printf("StartDeviceDA Error\n");
		_getch();
	}

	printf("press any key to Trigger device....\n");
	_getch();
	USB3020_SetDeviceTrigDA(hDevice, FALSE, nDAChannel);
	while(!_kbhit())
	{
		USB3020_GetDevStatusDA(hDevice, &DAStatus, nDAChannel);
		Sleep(10);
		printf("nCurSegNum=%2d  bTrigFlag=%4x  ",  DAStatus.nCurSegNum, DAStatus.bTrigFlag);
		printf("nCurLoopCount=%4d  nCurSegLoopCount=%3d TR=%1d\n",  DAStatus.nCurLoopCount, DAStatus.nCurSegLoopCount, DAStatus.bTrigFlag);
	}

	USB3020_DisableDeviceDA(hDevice, nDAChannel);		
	printf("Press any key to release device....\n");
	_getch();
	USB3020_ReleaseDeviceDA(hDevice, nDAChannel);   // 释放DA对象	

ExitOut:
	USB3020_ReleaseDevice(hDevice);   // 释放设备对象
	delete DABuffer;
	return 0;
	
}



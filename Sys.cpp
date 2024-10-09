// ��������ʾ��DA�������

#include "stdafx.h"
#include "windows.h"
#include "stdio.h"
#include "conio.h"
#include "math.h"
#include "stdlib.h"

#include "Wave.h"
#include "USB3020.h" // ��������ӿ�ͷ��(����)
#define ESC 27

#define BUFFER_SIZE 1024*1024*2


int main(int argc, char* argv[])
{
	HANDLE hDevice;  //  �豸������
	int DeviceID;    //  �豸��
	BOOL bStatus; // �����ķ���ֵ
	LONG nWriteOffsetWords=0; // �������������(��RAM)0λ�õ�ƫ�Ƶ�(˫��)
	LONG nWriteSizeWords;   // ��������ݳ���(˫��)
	LONG nRetSizeWords;
	LONG SegmentCount = 1;
	BOOL bResetDA = FALSE; // ÿ�ο�ʼʱ���Ƿ�RAMɨ��ָ�븴λ��0��0��ַ
	int WaveType = 0;
	float fTrigVolt = 1000.0; // ������ѹΪ1000����
	int SegmentIdx = 0;
	int Index = 0;
	USB3020_PARA_DA DAPara; // DA�����ṹ��
	USB3020_SEGMENT_INFO SegmentInfo[32];
	USB3020_STATUS_DA DAStatus; // DA״̬�ṹ��
	int SegmentAddr; // ��¼�ε�ַ
	PUSHORT DABuffer = new USHORT[BUFFER_SIZE];
	PUSHORT TDABuffer = new USHORT[BUFFER_SIZE];
	int nDAChannel = 0, SegmentID = 0;
	DeviceID = 0;
	hDevice = USB3020_CreateDevice( DeviceID ); // �����豸����
	if(hDevice == INVALID_HANDLE_VALUE) 
	{
		printf("Create Device Error...\n");
		_getch();
		return 0;
	}

	// �������ṹ���ʼ��Ϊȷ��ֵ0
	memset(&DAPara, 0, sizeof(DAPara));
	DAPara.OutputRange = USB3020_OUTPUT_N10000_P10000mV;
	DAPara.Frequency = 100000; // �ɼ�Ƶ��Hz
	DAPara.LoopCount = 0; // ����RAM�Ĵ�ѭ������,=0:����ѭ��, =n:��ʾn��ѭ��(n<2048)
	DAPara.TriggerSource = USB3020_TRIGSRC_SOFT_DA;
	DAPara.TriggerMode = USB3020_TRIGMODE_CONTINUOUS;		// ��/�ⴥ��(��AD����)��ʽѡ��
	DAPara.TriggerDir = USB3020_TRIGDIR_POSITIVE;	// ����/���򴥷�ѡ��
	DAPara.ClockSource = USB3020_CLOCKSRC_IN;		// ʱ��Դѡ��

	// �������ṹ���ʼ��Ϊȷ��ֵ0
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

	// ��ö�0��DABuffer�������еĵ�ַ
	SegmentAddr = 0;
	GenerateWave(&DABuffer[0], SegmentInfo[0].SegmentSize, 640, WAVE_PULSE); // ����0�ε�ַ�ͳ����������Ҳ�����

	// ��ö�1��DABuffer�������еĵ�ַ
	SegmentAddr = SegmentInfo[0].SegmentSize;
	GenerateWave(&DABuffer[SegmentAddr], SegmentInfo[1].SegmentSize, 640, WAVE_PULSE);

	// ��ö�2��DABuffer�������еĵ�ַ
	SegmentAddr = SegmentInfo[0].SegmentSize + SegmentInfo[1].SegmentSize;
	GenerateWave(&DABuffer[SegmentAddr], SegmentInfo[2].SegmentSize, 64, WAVE_PULSE);

	// ��ö�3��DABuffer�������еĵ�ַ
	SegmentAddr = SegmentInfo[0].SegmentSize + SegmentInfo[1].SegmentSize + SegmentInfo[2].SegmentSize;
	GenerateWave(&DABuffer[SegmentAddr], SegmentInfo[3].SegmentSize, 64, WAVE_PULSE);

	// ��ö�4��DABuffer�������еĵ�ַ
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
	USB3020_ReleaseDeviceDA(hDevice, nDAChannel);   // �ͷ�DA����	

ExitOut:
	USB3020_ReleaseDevice(hDevice);   // �ͷ��豸����
	delete DABuffer;
	return 0;
	
}



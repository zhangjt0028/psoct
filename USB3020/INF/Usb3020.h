#ifndef _USB3020_DEVICE_
#define _USB3020_DEVICE_

#include<windows.h>
//*************************************************************************************
// ������Ϣ
typedef struct _USB3020_SEGMENT_INFO
{
	LONG SegLoopCount;		// ÿ�����ڴ�ѭ���е�Сѭ������,ȡֵΪ[1, 16777215]
	LONG SegmentSize;		// ÿ������RAM�еĳ���(��λ����/��)
} USB3020_SEGMENT_INFO, *PUSB3020_SEGMENT_INFO;

// DA�Ĺ�������
typedef struct _USB3020_PARA_DA
{
	LONG OutputRange;		// �������
	LONG Frequency;         // ��Ƶ��[0.010Hz, 1MHz], Ϊ����ʱ��λHz��Ϊ����ʱ��λΪ0.001Hz
	LONG LoopCount;			// ����RAM�Ĵ�ѭ������,=0:����ѭ��, =n:��ʾn��ѭ��(1<n<32768)
	LONG TriggerMode;		// ����ģʽѡ��
	LONG TriggerSource;		// ����Դѡ��
	LONG TriggerDir;		// ��������ѡ��
	LONG bSingleOut;		// �Ƿ񵥵����
	LONG ClockSource;		// ʱ��Դѡ��
} USB3020_PARA_DA, *PUSB3020_PARA_DA;

//######################## �������� #################################
// DA�������InitDeviceDA��ģ���������Χ����OutputRange��ʹ�õ�ѡ��
const long USB3020_OUTPUT_N5000_P5000mV		= 0x00;		// ��5000mV
const long USB3020_OUTPUT_N10000_P10000mV	= 0x01;		// ��10000mV

// DAӲ������USB3020_PARA_DA�е�TriggerMode��Ա��������ģʽѡ��
const long USB3020_TRIGMODE_SINGLE			= 0x00;		// ���δ���
const long USB3020_TRIGMODE_CONTINUOUS		= 0x01;		// ��������
const long USB3020_TRIGMODE_STEPED			= 0x02;		// ��������
const long USB3020_TRIGMODE_BURST			= 0x03;		// ��������

// DAӲ������USB3020_PARA_DA�е�TriggerSource��Ա��������Դѡ��
const long USB3020_TRIGSRC_SOFT_DA			= 0x00;		// �������
const long USB3020_TRIGSRC_DTR_DA			= 0x01;		// DTRӲ��ģ�ⴥ��

//***********************************************************
// ADӲ������USB3020_PARA_AD�е�TriggerDir����������ʹ�õ�ѡ��
// DAӲ������USB3020_PARA_DA�е�TriggerDir��Ա��������ѡ��
const long USB3020_TRIGDIR_NEGATIVE			= 0x00;		// ���򴥷�(������/�½��ش���)
const long USB3020_TRIGDIR_POSITIVE			= 0x01;		// ���򴥷�(������/�����ش���)
const long USB3020_TRIGDIR_POSIT_NEGAT		= 0x02;		// �����򴥷�(��/�����������/�½��ش���)

//***********************************************************
// ADӲ������USB3020_PARA_AD�е�ClockSourceʱ��Դ��ʹ�õ�ѡ��
// DAӲ������USB3020_PARA_DA�е�ClockSource��Ա����ʱ��Դѡ��
const long USB3020_CLOCKSRC_IN				= 0x00;		// �ڲ�ʱ��
const long USB3020_CLOCKSRC_OUT				= 0x01;		// �ⲿʱ��

//*************************************************************************************
// ����DA������ʵ��Ӳ������
typedef struct _USB3020_STATUS_DA
{
	LONG bTrigFlag;		// ������־�Ƿ���Ч��=TRUE��ʾ�������Ч�� = FALSE��ʾ��Ч����������δ����
	LONG bConverting;	// DA�Ƿ�����ת���� =TRUE:��ʾ����ת���� = FALS��ʾת�����
	LONG nCurSegNum;	// �ɶ�ȡ��RAM�κţ�ȡֵΪ[0, SegmentCount-1], (עSegmentCountΪInitDeviceDA�����Ĳ���)
	LONG nCurSegAddr;	// �ɶ�ȡ��RAM�ε�ַ
	LONG nCurLoopCount; // ��ǰ��ѭ������
	LONG nCurSegLoopCount; // ��ǰ��ѭ������
} USB3020_STATUS_DA, *PUSB3020_STATUS_DA;

//***********************************************************
// ���ε�ͷ��Ϣ��RAM�еĵ�ַ�ռ�
const long USB3020_SEGMENT_HEADER_SIZE = 0x0C; // ��ͷ��Ϣ�ĳ���

//***********************************************************
// ���������ӿ�
#ifndef _USB3020_DRIVER_
#define DEVAPI __declspec(dllimport)
#else
#define DEVAPI __declspec(dllexport)
#endif

#ifdef __cplusplus
extern "C" {
#endif
	//######################## ����ͨ�ú��� #################################
    HANDLE DEVAPI FAR PASCAL USB3020_CreateDevice(int DeviceLgcID = 0); // �����豸����(�ú���ʹ��ϵͳ���߼��豸ID��
	int DEVAPI FAR PASCAL USB3020_GetDeviceCount(HANDLE hDevice);      // ȡ��USB3020��ϵͳ�е��豸����
	BOOL DEVAPI FAR PASCAL USB3020_GetDeviceCurrentID(HANDLE hDevice, PLONG DeviceLgcID, PLONG DevicePhysID); // ȡ�õ�ǰ�豸���߼�ID�ź�����ID��
	BOOL DEVAPI FAR PASCAL USB3020_ListDeviceDlg(void); // �öԻ����б�ϵͳ���е�����USB3020�豸
	BOOL DEVAPI FAR PASCAL USB3020_ResetDevice(HANDLE hDevice);		 // ��λ����USB�豸
    BOOL DEVAPI FAR PASCAL USB3020_ReleaseDevice(HANDLE hDevice);    // �豸���

	//####################### DA���ݶ�ȡ���� #################################
	// ���ڴ������ͨ�û�����Щ�ӿ���򵥡����ݡ���ɿ������û�����֪���豸
	// �Ͳ㸴�ӵ�Ӳ������Э��ͷ����������Ʊ�̣���������ĳ�ʼ���豸��
	// DA����������������������ɸ�Ч��ʵ�ָ��١������Ĳ������
	// DAͨ�ú���
	BOOL DEVAPI FAR PASCAL USB3020_InitDeviceDA(			// ��ʼ���豸��������TRUE��, �豸��׼������.
									HANDLE hDevice,			// �豸������,����CreateDevice��������
									LONG SegmentCount,		// �ܹ�������[1, 65536]
									USB3020_SEGMENT_INFO SegmentInfo[], // ����Ϣ����
									PUSB3020_PARA_DA pDAPara, // Ӳ������, �����ڴ˺����о���Ӳ��״̬
									int nDAChannel);		// DAͨ����[0, 3]

	BOOL DEVAPI FAR PASCAL USB3020_WriteDeviceBulkDA(		// ��DA���ǰ���ô˺�����DA����д�����RAM��(����ʽ)
									HANDLE hDevice,			// �豸������,����CreateDevice��������
									USHORT DABuffer[],		// Я��ԭʼDA���ݵ��û�������
									LONG nWriteSizeWords,   // �����ƫλ���д������ݳ���(��)
									PLONG nRetSizeWords,	// ����ʵ��д���ĳ���(��)
									int nDAChannel);		// DAͨ����[0, 3]	

    BOOL DEVAPI FAR PASCAL USB3020_EnableDeviceDA(			// �ڳ�ʼ��֮��ʹ���豸
									HANDLE hDevice,			// �豸������,����CreateDevice��������
									int nDAChannel);		// DAͨ����[0, 3]

    BOOL DEVAPI FAR PASCAL USB3020_SetDeviceTrigDA(			// ���豸ʹ������󣬲�����������¼���ֻ�д���ԴΪ�������ʱ��Ч��
									HANDLE hDevice,			// �豸������,����CreateDevice��������
									BOOL bSetSyncTrig,		// �Ƿ���ͬ������
									int nDAChannel);		// DAͨ����[0, 3]

	BOOL DEVAPI FAR PASCAL USB3020_GetDevStatusDA(			// ��DA����������ȡ���豸�ĸ���״̬,����ֵ��ʾ�����Ƿ�ɹ�
									HANDLE hDevice,			// �豸������,����CreateDevice��������
									PUSB3020_STATUS_DA pDAStatus, // DA�ĸ�����Ϣ�ṹ��
									int nDAChannel);		// DAͨ����[0, 3]

    BOOL DEVAPI FAR PASCAL USB3020_DisableDeviceDA(			// ��ʹ�豸֮�󣬽�ֹ�豸
									HANDLE hDevice,			// �豸������,����CreateDevice��������
									int nDAChannel);		// DAͨ����[0, 3]
	
	BOOL DEVAPI FAR PASCAL USB3020_ReleaseDeviceDA(			// ��ֹDA�豸, ���ͷ���Դ
									HANDLE hDevice,			// �豸������,����CreateDevice��������			
									int nDAChannel);		// DAͨ����[0, 3]

	// DAУ׼
	BOOL DEVAPI FAR PASCAL USB3020_StartCalibration(		// ����DAУ׼
									HANDLE hDevice);		// �豸����

	BOOL DEVAPI FAR PASCAL USB3020_GetDACalibration(		// �豸DAУ׼		
									HANDLE hDevice,			// �豸����
									LONG OutputRange,		// �������,�ֱ��������ͨ��
									LONG CalMode,			// 0Ϊ���У׼��1Ϊ����У׼
									PLONG pCalData,			// У׼ֵ
									int nDAChannel);		// DAͨ����[0, 3]

	BOOL DEVAPI FAR PASCAL USB3020_SetDACalibration(		// �豸DAУ׼		
									HANDLE hDevice,			// �豸����
									LONG OutputRange,		// �������,�ֱ��������ͨ��
									LONG CalMode,			// 0Ϊ���У׼��1Ϊ����У׼
									LONG CalData,			// У׼ֵ
									int nDAChannel);		// DAͨ����[0, 3]

	BOOL DEVAPI FAR PASCAL USB3020_StopCalibration(			// ֹͣDAУ׼
									HANDLE hDevice);		// �豸����
	
   	//################# DA��Ӳ�������������� ########################
	BOOL DEVAPI FAR PASCAL USB3020_SaveParaDA(HANDLE hDevice, PUSB3020_PARA_DA pDAPara, int nDAChannel);
    BOOL DEVAPI FAR PASCAL USB3020_LoadParaDA(HANDLE hDevice, PUSB3020_PARA_DA pDAPara, int nDAChannel);
    BOOL DEVAPI FAR PASCAL USB3020_ResetParaDA(HANDLE hDevice, PUSB3020_PARA_DA pDAPara, int nDAChannel);

	//############################################################################
	BOOL DEVAPI FAR PASCAL USB3020_GetDevVersion(				// ��ȡ�豸�̼�������汾
									HANDLE hDevice,				// �豸������,����CreateDevice��������
									PULONG pulFmwVersion,		// �̼��汾
									PULONG pulDriverVersion);	// �����汾

#ifdef __cplusplus
}
#endif

// �Զ������������������
#ifndef _USB3020_DRIVER_
#ifndef _WIN64
#pragma comment(lib, "USB3020_32.lib")
#pragma message("======== Welcome to use our art company's products!")
#pragma message("======== Automatically linking with USB3020_32.dll...")
#pragma message("======== Successfully linked with USB3020_32.dll")
#else
#pragma comment(lib, "USB3020_64.lib")
#pragma message("======== Welcome to use our art company's products!")
#pragma message("======== Automatically linking with USB3020_64.dll...")
#pragma message("======== Successfully linked with USB3020_64.dll")
#endif
#endif

#endif; // _USB3020_DEVICE_
#include "DAcard.h"
//#include "mainwidget.h"

DA_USB3020::DA_USB3020()
{
	Voltage = 1150;
	Frequency = 225.4;
	duty_cycle = 0.9;
	BScanlines = 800;
	SamplesPerSec = 4001000; 
	ZeroBufferPoint = 10; // 0V电压的点数
    xscanMode = 0;
    yscanMode = 2;

    pDataX = NULL;
    pDataY = NULL;
    pDataT = NULL;
    pSegmentInfoX = NULL;
    pSegmentInfoY = NULL;
    pSegmentInfoT = NULL;
    pParaDA_2D1D = NULL;
    pParaDA_3D = NULL;
    pParaDA_2D_Repeat = NULL;
    pParaDA_3D_Repeat = NULL;
    hDevice = INVALID_HANDLE_VALUE;
    curStatus = STATUS_STOP;

    ConnectDA();
}

void DA_USB3020::CalculateDAdata()
{
    len_Total = long(SamplesPerSec * 1.0 / Frequency + 0.5);
    len_Transit = long(SamplesPerSec * 0.5 * (1-duty_cycle) / Frequency + 0.5);
    len_Scan = len_Total - len_Transit * 2;

    delete [] pDataX;
    pDataX = new unsigned short[len_Total + ZeroBufferPoint];
    delete [] pDataY;
    pDataY = new unsigned short[TBD];
    delete [] pDataT;
    pDataT = new unsigned short[len_Total + ZeroBufferPoint];

    for (unsigned int i = 0; i < len_Total + ZeroBufferPoint; ++i)
    {
        pDataX[i] = 0;
        pDataT[i] = 0;
    }
    //  0->-V
    for (unsigned int i = 0; i < len_Transit; ++i)
    {
        pDataX[i] = LinearCut(Voltage_0V, Voltage_5V_neg, len_Transit, i);
        pDataT[i] = Voltage_0V;
    }
    // -V->V
    for (int i = 0; i < len_Scan; ++i)
    {
        pDataX[i + len_Transit] = LinearCut(Voltage_5V_neg, Voltage_5V, len_Scan, i);
        pDataT[i] = Voltage_5V;
    }
    // V->0
    for (unsigned int i = 0; i < len_Transit; ++i)
    {
        pDataX[i + len_Transit + len_Scan] = LinearCut(Voltage_5V, Voltage_0V, len_Transit, i);
        pDataT[i] = Voltage_0V;
    }
    // 0V
    for (unsigned int i = 0 ; i < ZeroBufferPoint; ++i)
    {
        pDataX[i + len_Total] = Voltage_0V;
        pDataT[i] = Voltage_0V;
    }


    delete [] pSegmentInfoX;
    delete [] pSegmentInfoY;
    delete [] pSegmentInfoT;

    //////////////not done///////////////
    pSegmentInfoX = new USB3020_SEGMENT_INFO;
    pSegmentInfoX[0].SegmentSize = len_Total + ZeroBufferPoint;
    pSegmentInfoX[0].SegLoopCount = BScanlines * AVERAGE_NUM;


    // TBscans
    for (int i = 0; i < BScanlines; ++i)
    {
        pDataY[2 * i] = pDataY[2 * i + 1] =
            short(32768 + Voltage * 65536.0 / 10000.0 *
            (2 * i - BScanlines + 1) / (BScanlines - 1) + 0.5);
    }

    pSegmentInfoY = new USB3020_SEGMENT_INFO[BScanlines];
    for (unsigned int i = 0; i < BScanlines; ++i)
    {
        pSegmentInfoY[i].SegmentSize = 2;
        pSegmentInfoY[i].SegLoopCount = len_RampScan / 2 * AVERAGE_NUM;
    }

    pSegmentInfoT = new USB3020_SEGMENT_INFO;
    pSegmentInfoT[0].SegmentSize = x_T1_len;
    pSegmentInfoT[0].SegLoopCount = 1;
    //////////////not done///////////////

        if (mode == MODE_2D_SCAN_REPEAT)
    {
        // Ѫ��OCT����ֶβ�������ά�ظ�
        pSegmentInfoX[0].SegmentSize = len_LinearScan + 2 * CosTransitPoint;
        pSegmentInfoX[0].SegLoopCount = 1;
        pSegmentInfoX[1].SegmentSize = x_T2_len + len_LinearScan + CosTransitPoint;
        pSegmentInfoX[1].SegLoopCount = 1;

        pSegmentInfoY[0].SegmentSize = CosTransitPoint;
        pSegmentInfoY[0].SegLoopCount = 1;
        pSegmentInfoY[1].SegmentSize = CosTransitPoint;
        pSegmentInfoY[1].SegLoopCount = 1;

        pSegmentInfoT[0].SegmentSize = len_LinearScan + 2 * CosTransitPoint;
        pSegmentInfoT[0].SegLoopCount = 1;
        pSegmentInfoT[1].SegmentSize = CosTransitPoint;
        pSegmentInfoT[1].SegLoopCount = 1;

    }
    else if (mode == MODE_3D_SCAN_REPEAT)
    {
        // Ѫ��OCT����ֶβ�������ά�ظ�
        int averageFrames = 4;
        pSegmentInfoX[2].SegLoopCount = BScanlines * averageFrames;
        for (unsigned int i = 0; i < BScanlines; ++i)
        {

            pSegmentInfoY[3 + i].SegLoopCount = len_RampScan / 2 * averageFrames;
            pSegmentInfoY[3 + i].SegmentSize = 2;
            pSegmentInfoY[3 + i].SegLoopCount = len_RampScan / 2 * averageFrames;
            pSegmentInfoT[2].SegLoopCount = BScanlines * averageFrames;
        }
    }

}

bool DA_USB3020::ConnectDA() ///
{
    hDevice = USB3020_CreateDevice(); // �����豸����
    if(hDevice == INVALID_HANDLE_VALUE)
        return false;
    else
    {
        delete pParaDA_2D1D;
        pParaDA_2D1D = new USB3020_PARA_DA();
        pParaDA_2D1D->OutputRange = USB3020_OUTPUT_N5000_P5000mV; // ��5V������źŷ�Χ
        pParaDA_2D1D->Frequency = SamplesPerSec;				// Ĭ�ϲ�����
        pParaDA_2D1D->LoopCount = 0;							// ��Ч
        pParaDA_2D1D->TriggerSource = USB3020_TRIGSRC_SOFT_DA;	// ��������
        pParaDA_2D1D->TriggerMode = USB3020_TRIGMODE_BURST;		// ��������
        pParaDA_2D1D->TriggerDir = USB3020_TRIGDIR_POSITIVE;	// �����ش���
        pParaDA_2D1D->ClockSource = USB3020_CLOCKSRC_OUT;		// �ڲ�ʱ��
        pParaDA_2D1D->bSingleOut = false;

        delete pParaDA_3D;
        pParaDA_3D = new USB3020_PARA_DA();
        pParaDA_3D->OutputRange = USB3020_OUTPUT_N5000_P5000mV; // ��5V������źŷ�Χ
        pParaDA_3D->Frequency = SamplesPerSec;					// Ĭ�ϲ�����
        pParaDA_3D->LoopCount = 0;								// ��Ч
        pParaDA_3D->TriggerSource = USB3020_TRIGSRC_SOFT_DA;	// ��������
        pParaDA_3D->TriggerMode = USB3020_TRIGMODE_SINGLE;		// ���δ���
        pParaDA_3D->TriggerDir = USB3020_TRIGDIR_POSITIVE;		// �����ش���
        pParaDA_3D->ClockSource = USB3020_CLOCKSRC_OUT;			// �ڲ�ʱ��
        pParaDA_3D->bSingleOut = false;

        // ��ʼ��Ĭ���ͷ�DA���������
        DisableDA();

        return true;
    }
}
///
bool DA_USB3020::WriteDataToDA()
{
    InitDAForScan(MODE_2D_CROSS_SCAN);
    long nRetSizeWords;
    nRetSizeWords = 0;
    USB3020_WriteDeviceBulkDA(hDevice, pDataX, x_len, &nRetSizeWords, xscanMode);

    nRetSizeWords = 0;
    USB3020_WriteDeviceBulkDA(hDevice, pDataT, x_len, &nRetSizeWords, 1 );

    nRetSizeWords = 0;
    USB3020_WriteDeviceBulkDA(hDevice, pDataY, y_len, &nRetSizeWords, yscanMode);

    return true;
}

bool DA_USB3020::InitDAForScan(int mode)
{
    PUSB3020_PARA_DA pPara = NULL;
    if (mode == MODE_1D_SCAN || mode == MODE_2D_CROSS_SCAN || mode == MODE_2D_SCAN_REPEAT)
    {
        pPara = pParaDA_2D1D;
    }
    else if (mode == MODE_3D_SCAN || mode == MODE_3D_SCAN_REPEAT)
    {
        pPara = pParaDA_3D;
    }
    curMode = mode;

    USB3020_InitDeviceDA(hDevice, 5, pSegmentInfoX, pPara, xscanMode); // DA0
    USB3020_InitDeviceDA(hDevice, 5, pSegmentInfoT, pPara, 1); // DA1
    USB3020_InitDeviceDA(hDevice, 6 + BScanlines, pSegmentInfoY, pPara, yscanMode); // DA3

    return true;

}

bool DA_USB3020::EnableDA()
{
    USB3020_EnableDeviceDA(hDevice, xscanMode);
    USB3020_EnableDeviceDA(hDevice, 1);
    USB3020_EnableDeviceDA(hDevice, yscanMode);
}

bool DA_USB3020::DisableDA() ///
{
    USB3020_DisableDeviceDA(hDevice, 0);
    USB3020_DisableDeviceDA(hDevice, 1);
    USB3020_DisableDeviceDA(hDevice, 2);
    USB3020_DisableDeviceDA(hDevice, 3);
    USB3020_ReleaseDeviceDA(hDevice, 0);
    USB3020_ReleaseDeviceDA(hDevice, 1);
    USB3020_ReleaseDeviceDA(hDevice, 2);
    USB3020_ReleaseDeviceDA(hDevice, 3);    
    return true;
}



bool DA_USB3020::isConnected() ///
{
    if(hDevice == INVALID_HANDLE_VALUE)
        return false;
    else
        return true;
}



bool DA_USB3020::Start2DscanRepeat()
{
    curMode = MODE_2D_SCAN_REPEAT;
    curStatus = STATUS_RUNNING;
    if (!InitDAForScan(MODE_2D_SCAN_REPEAT))
        return false;
    EnableDA();
    //USB3020_SetDeviceTrigDA(hDevice, false, 0);
    //USB3020_SetDeviceTrigDA(hDevice, false, 2);
    //Sleep(50);
    USB3020_SetDeviceTrigDA(hDevice, true, xscanMode);

    return true;
}

bool DA_USB3020::StopScan()
{
    if (curStatus == STATUS_RUNNING)
    {
        if (curMode == MODE_2D_CROSS_SCAN)
        {
            USB3020_SetDeviceTrigDA(hDevice, true, xscanMode);
            Sleep(100);
        }
        else if (curMode == MODE_2D_SCAN_REPEAT)
        {
            USB3020_SetDeviceTrigDA(hDevice, true, xscanMode);
            Sleep(100);
        }
        else if(curMode == MODE_1D_SCAN)
        {
            USB3020_SetDeviceTrigDA(hDevice, false, 1 );
            Sleep(100);
        }
        else if (curMode == MODE_3D_SCAN)
        {
            USB3020_GetDevStatusDA(hDevice, &m_DAStatus, yscanMode);
            if (m_DAStatus.nCurSegNum < BScanlines + 4)
                return false;
        }
        DisableDA();
        curStatus = STATUS_STOP;
        return true;
    }
    return false;
}

bool DA_USB3020::Start3DscanRepeat()
{
    curMode = MODE_3D_SCAN_REPEAT;
    curStatus = STATUS_RUNNING;
    if (!InitDAForScan(MODE_3D_SCAN_REPEAT))
        return false;
    EnableDA()
    USB3020_SetDeviceTrigDA(hDevice, true, xscanMode);
    return true;
}

DA_USB3020::~DA_USB3020()
{
    DisableDA();
    USB3020_ReleaseDevice(hDevice);   
    // 注意是不是数组 // not done //
    delete [] pSegmentInfoX;
    delete [] pSegmentInfoY;
    delete [] pSegmentInfoT;
    delete pParaDA_2D1D;
    delete pParaDA_3D;
    delete [] pDataX;
    delete [] pDataY;
    delete [] pDataT;

}

short LinearCut(unsigned int start, unsigned int end, unsigned long length, unsigned int i){
    return short(start + (end - start) / (length - 1) * i + 0.5);
}


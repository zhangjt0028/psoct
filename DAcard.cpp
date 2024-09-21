#include "DAcard.h"
//#include "mainwidget.h"

DA_USB3020::DA_USB3020(unsigned long SamplesPerSec, unsigned long ZeroBufferPoint, unsigned long CosTransitPoint, int xscanMode, int yscanMode)
{
    this->SamplesPerSec = SamplesPerSec;
    this->ZeroBufferPoint = ZeroBufferPoint;
    this->CosTransitPoint = CosTransitPoint;
    this->xscanMode = xscanMode;
    this->yscanMode = yscanMode;

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

void DA_USB3020::CalculateDAdata(unsigned int Voltage, double Frequency, double duty_cycle, unsigned int BScanlines,int xscanMode, int yscanMode)
{
    this->Voltage = Voltage;
    this->Frequency = Frequency;
    this->duty_cycle = duty_cycle;
    this->BScanlines = BScanlines;
    this->xscanMode = xscanMode;
    this->yscanMode = yscanMode;


    GenDataX();
    GenDataY();
    GenDataT();
}

void DA_USB3020::CalculateLength(unsigned int Voltage, double Frequency, double duty_cycle, unsigned int BScanlines)
{
    this->Voltage = Voltage;
    this->Frequency = Frequency;
    this->duty_cycle = duty_cycle;
    this->BScanlines = BScanlines;
    //锯齿波一个周期的点数
    len_RampScan = long(SamplesPerSec * 1.0 / Frequency + 0.5);
    // 保证一个周期的长度能够被2整除
    if (len_RampScan % 2 != 0)
    {
        len_RampScan = len_RampScan - 1;
    }
    // 线性扫描区域的长度
    len_LinearScan = long(SamplesPerSec * duty_cycle / Frequency + 0.5);
    CosTransitPoint = long(SamplesPerSec * 0.5 * (1-duty_cycle) / Frequency + 0.5);
    //////////////////////////////////////////////////////////////////////////
    // X通道总共分为4段，每段的起始点和长度分别如下所示
    //////////////////////////////////////////////////////////////////////////
    x_T1_beg = 0;
    x_T1_len = 2 * len_LinearScan + 3 * CosTransitPoint;
    x_T1_1_beg = 0;
    x_T1_1_len = CosTransitPoint;
    x_T1_2_beg = x_T1_1_beg + x_T1_1_len;
    x_T1_2_len = len_LinearScan;
    x_T1_3_beg = x_T1_2_beg + x_T1_2_len;
    x_T1_3_len = CosTransitPoint;
    x_T1_4_beg = x_T1_3_beg + x_T1_3_len;
    x_T1_4_len = len_LinearScan;
    x_T1_5_beg = x_T1_4_beg + x_T1_4_len;
    x_T1_5_len = CosTransitPoint;

    x_T2_beg = x_T1_beg + x_T1_len;
    x_T2_len = ZeroBufferPoint;

    x_T3_beg = x_T2_beg + x_T2_len;
    x_T3_len = len_RampScan;
    x_T3_1_beg = x_T3_beg;
    x_T3_1_len = (len_RampScan - len_LinearScan) / 2;
    x_T3_2_beg = x_T3_1_beg + x_T3_1_len;
    x_T3_2_len = len_LinearScan;
    x_T3_3_beg = x_T3_2_beg + x_T3_2_len;
    x_T3_3_len = len_RampScan - len_LinearScan - x_T3_1_len;

    x_T4_beg = x_T3_beg + x_T3_len;
    x_T4_len = ZeroBufferPoint;
    x_len = x_T1_len + x_T2_len + x_T3_len + x_T4_len;

    y_T1_beg = 0;
    y_T1_len = 2 * len_LinearScan + 3 * CosTransitPoint;
    y_T1_1_beg = 0;
    y_T1_1_len = CosTransitPoint;
    y_T1_2_beg = y_T1_1_beg + y_T1_1_len;
    y_T1_2_len = len_LinearScan;
    y_T1_3_beg = y_T1_2_beg + y_T1_2_len;
    y_T1_3_len = CosTransitPoint;
    y_T1_4_beg = y_T1_3_beg + y_T1_3_len;
    y_T1_4_len = len_LinearScan;
    y_T1_5_beg = y_T1_4_beg + y_T1_4_len;
    y_T1_5_len = CosTransitPoint;

    y_T2_beg = y_T1_beg + y_T1_len;
    y_T2_len = ZeroBufferPoint;

    y_T3_beg = y_T2_beg + y_T2_len;
    y_T3_len = CosTransitPoint;

    y_TBscans_beg = y_T3_beg + y_T3_len;
    y_TBScans_len = 2 * BScanlines;

    y_T4_beg = y_TBscans_beg + y_TBScans_len;
    y_T4_len = CosTransitPoint;

    y_T5_beg = y_T4_beg + y_T4_len;
    y_T5_len = ZeroBufferPoint;

    y_len = y_T1_len + y_T2_len + y_T3_len + 2 * BScanlines + y_T4_len + y_T5_len;

    if (pSegmentInfoX)
        delete [] pSegmentInfoX;

    pSegmentInfoX = new USB3020_SEGMENT_INFO[5];
    pSegmentInfoX[0].SegmentSize = x_T1_len;
    pSegmentInfoX[0].SegLoopCount = 1;
    pSegmentInfoX[1].SegmentSize = x_T2_len;
    pSegmentInfoX[1].SegLoopCount = CosTransitPoint / ZeroBufferPoint + 1;
    pSegmentInfoX[2].SegmentSize = x_T3_len;
    pSegmentInfoX[2].SegLoopCount = BScanlines * AVERAGE_NUM;
    pSegmentInfoX[3].SegmentSize = x_T4_len - 2;
    pSegmentInfoX[3].SegLoopCount = 16777215;
    pSegmentInfoX[4].SegmentSize = 2;
    pSegmentInfoX[4].SegLoopCount = 1;	// 无效

    if (pSegmentInfoY)
        delete [] pSegmentInfoY;
    pSegmentInfoY = new USB3020_SEGMENT_INFO[6 + BScanlines];
    pSegmentInfoY[0].SegmentSize = y_T1_len;
    pSegmentInfoY[0].SegLoopCount = 1;
    pSegmentInfoY[1].SegmentSize = y_T2_len;
    pSegmentInfoY[1].SegLoopCount = 1;
    pSegmentInfoY[2].SegmentSize = y_T3_len;
    pSegmentInfoY[2].SegLoopCount = 1;
    for (unsigned int i = 0; i < BScanlines; ++i)
    {
        pSegmentInfoY[3 + i].SegmentSize = 2;
        pSegmentInfoY[3 + i].SegLoopCount = len_RampScan / 2 * AVERAGE_NUM;
    }
    pSegmentInfoY[3 + BScanlines].SegmentSize = y_T4_len;
    pSegmentInfoY[3 + BScanlines].SegLoopCount = 1;
    pSegmentInfoY[4 + BScanlines].SegmentSize = y_T5_len - 2;
    pSegmentInfoY[4 + BScanlines].SegLoopCount = 16777215;
    pSegmentInfoY[5 + BScanlines].SegmentSize = 2;
    pSegmentInfoY[5 + BScanlines].SegLoopCount = 1;

    if (pSegmentInfoT)
        delete [] pSegmentInfoT;
    pSegmentInfoT = new USB3020_SEGMENT_INFO[5];
    pSegmentInfoT[0].SegmentSize = x_T1_len;
    pSegmentInfoT[0].SegLoopCount = 1;
    pSegmentInfoT[1].SegmentSize = x_T2_len;
    pSegmentInfoT[1].SegLoopCount = CosTransitPoint / ZeroBufferPoint + 1;
    pSegmentInfoT[2].SegmentSize = x_T3_len;
    pSegmentInfoT[2].SegLoopCount = BScanlines * AVERAGE_NUM;
    pSegmentInfoT[3].SegmentSize = x_T4_len - 2;
    pSegmentInfoT[3].SegLoopCount = 16777215;
    pSegmentInfoT[4].SegmentSize = 2;
    pSegmentInfoT[4].SegLoopCount = 1;	// 无效

}

bool DA_USB3020::ConnectDA()
{
    hDevice = USB3020_CreateDevice(); // 创建设备对象
    if(hDevice == INVALID_HANDLE_VALUE)
        return false;
    else
    {
        if (pParaDA_2D1D)
            delete pParaDA_2D1D;
        pParaDA_2D1D = new USB3020_PARA_DA();
        pParaDA_2D1D->OutputRange = USB3020_OUTPUT_N5000_P5000mV; // ±5V的输出信号范围
        pParaDA_2D1D->Frequency = SamplesPerSec;				// 默认采样率
        pParaDA_2D1D->LoopCount = 0;							// 无效
        pParaDA_2D1D->TriggerSource = USB3020_TRIGSRC_SOFT_DA;	// 软件触发
        pParaDA_2D1D->TriggerMode = USB3020_TRIGMODE_BURST;		// 紧急触发
        pParaDA_2D1D->TriggerDir = USB3020_TRIGDIR_POSITIVE;	// 上升沿触发
        pParaDA_2D1D->ClockSource = USB3020_CLOCKSRC_OUT;		// 内部时钟
        pParaDA_2D1D->bSingleOut = false;

        if (pParaDA_3D)
            delete pParaDA_3D;
        pParaDA_3D = new USB3020_PARA_DA();
        pParaDA_3D->OutputRange = USB3020_OUTPUT_N5000_P5000mV; // ±5V的输出信号范围
        pParaDA_3D->Frequency = SamplesPerSec;					// 默认采样率
        pParaDA_3D->LoopCount = 0;								// 无效
        pParaDA_3D->TriggerSource = USB3020_TRIGSRC_SOFT_DA;	// 软件触发
        pParaDA_3D->TriggerMode = USB3020_TRIGMODE_SINGLE;		// 单次触发
        pParaDA_3D->TriggerDir = USB3020_TRIGDIR_POSITIVE;		// 上升沿触发
        pParaDA_3D->ClockSource = USB3020_CLOCKSRC_OUT;			// 内部时钟
        pParaDA_3D->bSingleOut = false;

        // 初始化默认释放DA，避免错误
        DisableDA();
        ReleaseDA();

        return true;
    }
}

bool DA_USB3020::WriteDataToDA()
{
    InitDAForScan(MODE_2D_CROSS_SCAN, false);
    long nRetSizeWords;
    nRetSizeWords = 0;
    bStatus = USB3020_WriteDeviceBulkDA(hDevice, pDataX, x_len,
        &nRetSizeWords, xscanMode);
    if(!bStatus)
    {
        return false;
    }

    nRetSizeWords = 0;
    bStatus = USB3020_WriteDeviceBulkDA(hDevice, pDataT, x_len,
        &nRetSizeWords, 1 );
    if(!bStatus)
    {
        return false;
    }

    nRetSizeWords = 0;
    bStatus = USB3020_WriteDeviceBulkDA(hDevice, pDataY, y_len,
        &nRetSizeWords, yscanMode);
    if(!bStatus)
    {
        return false;
    }


    return true;
}

bool DA_USB3020::InitDAForScan(int mode, bool thenEnableDA)
{
    if (!isReadyForScan())
        return false;
    PUSB3020_PARA_DA pPara = NULL;
    if (mode == MODE_1D_SCAN || mode == MODE_2D_CROSS_SCAN || mode == MODE_2D_SCAN_REPEAT)
    {
        pPara = pParaDA_2D1D;
    }
    else if (mode == MODE_3D_SCAN || mode == MODE_3D_SCAN_REPEAT)
    {
        pPara = pParaDA_3D;
    }
    else
    {
        return false;
    }
    curMode = mode;


    // 结构OCT计算分段参数
    //锯齿波一个周期的点数
    len_RampScan = long(SamplesPerSec * 1.0 / Frequency + 0.5);
    // 保证一个周期的长度能够被2整除
    if (len_RampScan % 2 != 0)
    {
        len_RampScan = len_RampScan - 1;
    }
    // 线性扫描区域的长度
    len_LinearScan = long(SamplesPerSec * duty_cycle / Frequency + 0.5);
    CosTransitPoint = long(SamplesPerSec * 0.5 * (1-duty_cycle) / Frequency + 0.5);
    //////////////////////////////////////////////////////////////////////////
    // X通道总共分为4段，每段的起始点和长度分别如下所示
    //////////////////////////////////////////////////////////////////////////
    x_T1_beg = 0;
    x_T1_len = 2 * len_LinearScan + 3 * CosTransitPoint;
    x_T1_1_beg = 0;
    x_T1_1_len = CosTransitPoint;
    x_T1_2_beg = x_T1_1_beg + x_T1_1_len;
    x_T1_2_len = len_LinearScan;
    x_T1_3_beg = x_T1_2_beg + x_T1_2_len;
    x_T1_3_len = CosTransitPoint;
    x_T1_4_beg = x_T1_3_beg + x_T1_3_len;
    x_T1_4_len = len_LinearScan;
    x_T1_5_beg = x_T1_4_beg + x_T1_4_len;
    x_T1_5_len = CosTransitPoint;

    x_T2_beg = x_T1_beg + x_T1_len;
    x_T2_len = ZeroBufferPoint;

    x_T3_beg = x_T2_beg + x_T2_len;
    x_T3_len = len_RampScan;
    x_T3_1_beg = x_T3_beg;
    x_T3_1_len = (len_RampScan - len_LinearScan) / 2;
    x_T3_2_beg = x_T3_1_beg + x_T3_1_len;
    x_T3_2_len = len_LinearScan;
    x_T3_3_beg = x_T3_2_beg + x_T3_2_len;
    x_T3_3_len = len_RampScan - len_LinearScan - x_T3_1_len;

    x_T4_beg = x_T3_beg + x_T3_len;
    x_T4_len = ZeroBufferPoint;
    x_len = x_T1_len + x_T2_len + x_T3_len + x_T4_len;

    y_T1_beg = 0;
    y_T1_len = 2 * len_LinearScan + 3 * CosTransitPoint;
    y_T1_1_beg = 0;
    y_T1_1_len = CosTransitPoint;
    y_T1_2_beg = y_T1_1_beg + y_T1_1_len;
    y_T1_2_len = len_LinearScan;
    y_T1_3_beg = y_T1_2_beg + y_T1_2_len;
    y_T1_3_len = CosTransitPoint;
    y_T1_4_beg = y_T1_3_beg + y_T1_3_len;
    y_T1_4_len = len_LinearScan;
    y_T1_5_beg = y_T1_4_beg + y_T1_4_len;
    y_T1_5_len = CosTransitPoint;

    y_T2_beg = y_T1_beg + y_T1_len;
    y_T2_len = ZeroBufferPoint;

    y_T3_beg = y_T2_beg + y_T2_len;
    y_T3_len = CosTransitPoint;

    y_TBscans_beg = y_T3_beg + y_T3_len;
    y_TBScans_len = 2 * BScanlines;

    y_T4_beg = y_TBscans_beg + y_TBScans_len;
    y_T4_len = CosTransitPoint;

    y_T5_beg = y_T4_beg + y_T4_len;
    y_T5_len = ZeroBufferPoint;

    y_len = y_T1_len + y_T2_len + y_T3_len + 2 * BScanlines + y_T4_len + y_T5_len;

    if (pSegmentInfoX)
        delete [] pSegmentInfoX;

    pSegmentInfoX = new USB3020_SEGMENT_INFO[5];
    pSegmentInfoX[0].SegmentSize = x_T1_len;
    pSegmentInfoX[0].SegLoopCount = 1;
    pSegmentInfoX[1].SegmentSize = x_T2_len;
    pSegmentInfoX[1].SegLoopCount = CosTransitPoint / ZeroBufferPoint + 1;
    pSegmentInfoX[2].SegmentSize = x_T3_len;
    pSegmentInfoX[2].SegLoopCount = BScanlines * AVERAGE_NUM;
    pSegmentInfoX[3].SegmentSize = x_T4_len - 2;
    pSegmentInfoX[3].SegLoopCount = 16777215;
    pSegmentInfoX[4].SegmentSize = 2;
    pSegmentInfoX[4].SegLoopCount = 1;	// 无效

    if (pSegmentInfoY)
        delete [] pSegmentInfoY;
    pSegmentInfoY = new USB3020_SEGMENT_INFO[6 + BScanlines];
    pSegmentInfoY[0].SegmentSize = y_T1_len;
    pSegmentInfoY[0].SegLoopCount = 1;
    pSegmentInfoY[1].SegmentSize = y_T2_len;
    pSegmentInfoY[1].SegLoopCount = 1;
    pSegmentInfoY[2].SegmentSize = y_T3_len;
    pSegmentInfoY[2].SegLoopCount = 1;
    for (unsigned int i = 0; i < BScanlines; ++i)
    {
        pSegmentInfoY[3 + i].SegmentSize = 2;
        pSegmentInfoY[3 + i].SegLoopCount = len_RampScan / 2 * AVERAGE_NUM;
    }
    pSegmentInfoY[3 + BScanlines].SegmentSize = y_T4_len;
    pSegmentInfoY[3 + BScanlines].SegLoopCount = 1;
    pSegmentInfoY[4 + BScanlines].SegmentSize = y_T5_len - 2;
    pSegmentInfoY[4 + BScanlines].SegLoopCount = 16777215;
    pSegmentInfoY[5 + BScanlines].SegmentSize = 2;
    pSegmentInfoY[5 + BScanlines].SegLoopCount = 1;

    if (pSegmentInfoT)
        delete [] pSegmentInfoT;
    pSegmentInfoT = new USB3020_SEGMENT_INFO[5];
    pSegmentInfoT[0].SegmentSize = x_T1_len;
    pSegmentInfoT[0].SegLoopCount = 1;
    pSegmentInfoT[1].SegmentSize = x_T2_len;
    pSegmentInfoT[1].SegLoopCount = CosTransitPoint / ZeroBufferPoint + 1;
    pSegmentInfoT[2].SegmentSize = x_T3_len;
    pSegmentInfoT[2].SegLoopCount = BScanlines * AVERAGE_NUM;
    pSegmentInfoT[3].SegmentSize = x_T4_len - 2;
    pSegmentInfoT[3].SegLoopCount = 16777215;
    pSegmentInfoT[4].SegmentSize = 2;
    pSegmentInfoT[4].SegLoopCount = 1;	// 无效

    if (mode == MODE_2D_SCAN_REPEAT)
    {
        // 血流OCT计算分段参数，二维重复
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
        // 血流OCT计算分段参数，三维重复
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


    bStatus = USB3020_InitDeviceDA(hDevice, 5, pSegmentInfoX, pPara, xscanMode); // DA0
    if (!bStatus)
    {
        return false;
    }


    bStatus = USB3020_InitDeviceDA(hDevice, 5, pSegmentInfoT, pPara, 1); // DA1
    if (!bStatus)
    {
        return false;
    }

    bStatus = USB3020_InitDeviceDA(hDevice, 6 + BScanlines,
        pSegmentInfoY, pPara, yscanMode); // DA3
    if (!bStatus)
    {
        return false;
    }


    if (thenEnableDA)
        EnableDA();
    return true;

}

bool DA_USB3020::EnableDA()
{
    if(!USB3020_EnableDeviceDA(hDevice, xscanMode))
    {
        return false;
    }
    if(!USB3020_EnableDeviceDA(hDevice, 1))
    {
        return false;
    }
    if(!USB3020_EnableDeviceDA(hDevice, yscanMode))
    {
        return false;
    }

}

bool DA_USB3020::DisableDA()
{
    USB3020_DisableDeviceDA(hDevice, 0);
    USB3020_DisableDeviceDA(hDevice, 1);
    USB3020_DisableDeviceDA(hDevice, 2);
    USB3020_DisableDeviceDA(hDevice, 3);
    return true;
}

bool DA_USB3020::ReleaseDA()
{
    USB3020_ReleaseDeviceDA(hDevice, 0);
    USB3020_ReleaseDeviceDA(hDevice, 1);
    USB3020_ReleaseDeviceDA(hDevice, 2);
    USB3020_ReleaseDeviceDA(hDevice, 3);
    return true;
}

bool DA_USB3020::isConnected()
{
    if(hDevice == INVALID_HANDLE_VALUE)
        return false;
    else
        return true;
}

bool DA_USB3020::isReadyForScan()
{
    if (isConnected() && pSegmentInfoX != NULL)
        return true;
    else
        return false;

}

bool DA_USB3020::Start1Dscan()
{
    curMode = MODE_1D_SCAN;
    curStatus = STATUS_RUNNING;
    if (!InitDAForScan(MODE_1D_SCAN))
        return false;
    USB3020_SetDeviceTrigDA(hDevice, false, xscanMode);
    USB3020_SetDeviceTrigDA(hDevice, false, 2);
    Sleep(50);
    USB3020_SetDeviceTrigDA(hDevice, true, yscanMode);

    return true;

}

bool DA_USB3020::Start2Dscan()
{
    curMode = MODE_2D_CROSS_SCAN;
    curStatus = STATUS_RUNNING;
    if (!InitDAForScan(MODE_2D_CROSS_SCAN))
        return false;
    if (!USB3020_SetDeviceTrigDA(hDevice, true, xscanMode))
        return false;
    return true;
}

bool DA_USB3020::Start2DscanRepeat()
{
    curMode = MODE_2D_SCAN_REPEAT;
    curStatus = STATUS_RUNNING;
    if (!InitDAForScan(MODE_2D_SCAN_REPEAT))
        return false;
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
        ReleaseDA();
        curStatus = STATUS_STOP;
        return true;
    }
    return false;
}

void DA_USB3020::GetDAstatus(int channel, long& bTrigFlag, long& bConverting,
                             long& nCurSegNum, long& nCurSegAddr,
                             long& nCurLoopCount, long& nCurSegLoopCount)
{
    USB3020_GetDevStatusDA(hDevice, &m_DAStatus, channel);
    bTrigFlag = m_DAStatus.bTrigFlag;
    bConverting = m_DAStatus.bConverting;
    nCurSegNum = m_DAStatus.nCurSegNum;
    if (channel == 0 || channel == 1)
        nCurSegNum += BScanlines + 1;
    nCurSegAddr = m_DAStatus.nCurSegAddr;
    nCurLoopCount = m_DAStatus.nCurLoopCount;
    nCurSegLoopCount = m_DAStatus.nCurSegLoopCount;
}

bool DA_USB3020::Start3Dscan()
{
    curMode = MODE_3D_SCAN;
    curStatus = STATUS_RUNNING;
    if (!InitDAForScan(MODE_3D_SCAN))
        return false;
    USB3020_SetDeviceTrigDA(hDevice, true, xscanMode);
    return true;
}

bool DA_USB3020::Start3DscanRepeat()
{
    curMode = MODE_3D_SCAN_REPEAT;
    curStatus = STATUS_RUNNING;
    if (!InitDAForScan(MODE_3D_SCAN_REPEAT))
        return false;
    USB3020_SetDeviceTrigDA(hDevice, true, xscanMode);
    return true;
}

DA_USB3020::~DA_USB3020()
{
    DisableDA();
    ReleaseDA();
    USB3020_ReleaseDevice(hDevice);   // 释放设备对象

    if (pSegmentInfoX)
        delete [] pSegmentInfoX;

    if (pSegmentInfoY)
        delete [] pSegmentInfoY;

    if (pSegmentInfoT)
        delete [] pSegmentInfoT;

    if (pParaDA_2D1D)
        delete pParaDA_2D1D;
    if (pParaDA_3D)
        delete pParaDA_3D;

    if (pDataX)
        delete [] pDataX;

    if (pDataY)
        delete [] pDataY;

    if (pDataT)
        delete [] pDataT;

}

void DA_USB3020::GenDataX()
{
    //锯齿波一个周期的点数
    len_RampScan = long(SamplesPerSec * 1.0 / Frequency + 0.5);
    // 保证一个周期的长度能够被2整除
    if (len_RampScan % 2 != 0)
    {
        len_RampScan = len_RampScan - 1;
    }
    // 线性扫描区域的长度
    len_LinearScan = long(SamplesPerSec * duty_cycle / Frequency + 0.5);
    CosTransitPoint = long(SamplesPerSec * 0.5 * (1-duty_cycle) / Frequency + 0.5);
    //////////////////////////////////////////////////////////////////////////
    // X通道总共分为4段，每段的起始点和长度分别如下所示
    //////////////////////////////////////////////////////////////////////////
    x_T1_beg = 0;
    x_T1_len = 2 * len_LinearScan + 3 * CosTransitPoint;
    x_T1_1_beg = 0;
    x_T1_1_len = CosTransitPoint;
    x_T1_2_beg = x_T1_1_beg + x_T1_1_len;
    x_T1_2_len = len_LinearScan;
    x_T1_3_beg = x_T1_2_beg + x_T1_2_len;
    x_T1_3_len = CosTransitPoint;
    x_T1_4_beg = x_T1_3_beg + x_T1_3_len;
    x_T1_4_len = len_LinearScan;
    x_T1_5_beg = x_T1_4_beg + x_T1_4_len;
    x_T1_5_len = CosTransitPoint;

    x_T2_beg = x_T1_beg + x_T1_len;
    x_T2_len = ZeroBufferPoint;

    x_T3_beg = x_T2_beg + x_T2_len;
    x_T3_len = len_RampScan;
    x_T3_1_beg = x_T3_beg;
    x_T3_1_len = (len_RampScan - len_LinearScan) / 2;
    x_T3_2_beg = x_T3_1_beg + x_T3_1_len;
    x_T3_2_len = len_LinearScan;
    x_T3_3_beg = x_T3_2_beg + x_T3_2_len;
    x_T3_3_len = len_RampScan - len_LinearScan - x_T3_1_len;

    x_T4_beg = x_T3_beg + x_T3_len;
    x_T4_len = ZeroBufferPoint;
    x_len = x_T1_len + x_T2_len + x_T3_len + x_T4_len;

    if (pDataX)
        delete [] pDataX;
    pDataX = new unsigned short[x_len];

    for (unsigned int i = 0; i < x_len; ++i)
    {
        pDataX[i] = 0;
    }

    // x_T1_1: 0->-V CosTransit
    for (unsigned int i = 0; i < x_T1_1_len; ++i)
    {
        pDataX[i + x_T1_1_beg] = short(32768 + 65536.0 * Voltage * 0.5 /
            10000.0 * (cos(i * M_PI / (x_T1_1_len - 1)) - 1) + 0.5);
    }

    // x_T1_2: LinearScan -V->V
    for (int i = 0; i < x_T1_2_len; ++i)
    {
        pDataX[i + x_T1_2_beg] = short(32768 + Voltage * 65536.0 / 10000.0 *
            (2 * i - x_T1_2_len + 1) / (x_T1_2_len - 1) + 0.5);
    }

    // x_T1_3: V->0 CosTransit
    for (unsigned int i = 0; i < x_T1_3_len; ++i)
    {
        pDataX[i + x_T1_3_beg] = short(32768 + 65536.0 * Voltage * 0.5 /
            10000.0 * (cos(i * M_PI / (x_T1_1_len - 1)) + 1) + 0.5);
    }

    // x_T1_4 & x_T1_5 0V
    for (unsigned int i = 0 ; i < x_T1_4_len + x_T1_5_len; ++i)
    {
        pDataX[i + x_T1_4_beg] = Voltage_0V;
    }

    // x_T2 0V
    for (unsigned int i = 0; i < x_T2_len; ++i)
    {
        pDataX[i + x_T2_beg] = Voltage_0V;
    }

    // x_T3_1
    for (unsigned int i = 0; i < x_T3_1_len; ++i)
    {
        pDataX[i + x_T3_1_beg] = short(32768 + Voltage * 65536.0 / 10000.0 *
            cos(M_PI / 2 * (x_T3_1_len + i + 1) / x_T3_1_len) + 0.5);
    }

    // x_T3_2
    for (int i = 0; i < x_T3_2_len; ++i)
    {
        pDataX[i + x_T3_2_beg] = short(32768 + Voltage * 65536.0 / 10000.0 *
            (2 * i - x_T3_2_len + 1) / (x_T3_2_len - 1) + 0.5);
    }

    // x_T3_3
    for (unsigned int i = 0; i < x_T3_3_len; ++i)
    {
        pDataX[i + x_T3_3_beg] = short(32768 + Voltage * 65536.0 / 10000.0 *
            cos(M_PI / 2 * i / (x_T3_3_len - 1)) + 0.5);
    }

    // x_T4
    for (unsigned int i = 0; i < x_T4_len; ++i)
    {
        pDataX[i + x_T4_beg] = Voltage_0V;
    }

    // 写入扫描信号到数据
    std::ofstream file1("scanX.txt");
    for (unsigned int i = 0; i < x_len; ++i)
    {
        file1 << pDataX[i] << std::endl;
    }
    file1.close();

    if (pSegmentInfoX)
        delete [] pSegmentInfoX;

    pSegmentInfoX = new USB3020_SEGMENT_INFO[5];
    pSegmentInfoX[0].SegmentSize = x_T1_len;
    pSegmentInfoX[0].SegLoopCount = 1;
    pSegmentInfoX[1].SegmentSize = x_T2_len;
    pSegmentInfoX[1].SegLoopCount = CosTransitPoint / ZeroBufferPoint + 1;
    pSegmentInfoX[2].SegmentSize = x_T3_len;
    pSegmentInfoX[2].SegLoopCount = BScanlines * AVERAGE_NUM;
    pSegmentInfoX[3].SegmentSize = x_T4_len - 2;
    pSegmentInfoX[3].SegLoopCount = 16777215;
    pSegmentInfoX[4].SegmentSize = 2;
    pSegmentInfoX[4].SegLoopCount = 1;	// 无效

}

void DA_USB3020::GenDataY()
{
    //////////////////////////////////////////////////////////////////////////
    // Y通道总共分为5 + Bscan段，Bscan每段长度为2，剩余5段长度分别如下
    //////////////////////////////////////////////////////////////////////////
    CosTransitPoint = long(SamplesPerSec * 0.5 * (1-duty_cycle) / Frequency + 0.5);
    y_T1_beg = 0;
    y_T1_len = 2 * len_LinearScan + 3 * CosTransitPoint;
    y_T1_1_beg = 0;
    y_T1_1_len = CosTransitPoint;
    y_T1_2_beg = y_T1_1_beg + y_T1_1_len;
    y_T1_2_len = len_LinearScan;
    y_T1_3_beg = y_T1_2_beg + y_T1_2_len;
    y_T1_3_len = CosTransitPoint;
    y_T1_4_beg = y_T1_3_beg + y_T1_3_len;
    y_T1_4_len = len_LinearScan;
    y_T1_5_beg = y_T1_4_beg + y_T1_4_len;
    y_T1_5_len = CosTransitPoint;

    y_T2_beg = y_T1_beg + y_T1_len;
    y_T2_len = ZeroBufferPoint;

    y_T3_beg = y_T2_beg + y_T2_len;
    y_T3_len = CosTransitPoint;

    y_TBscans_beg = y_T3_beg + y_T3_len;
    y_TBScans_len = 2 * BScanlines;

    y_T4_beg = y_TBscans_beg + y_TBScans_len;
    y_T4_len = CosTransitPoint;

    y_T5_beg = y_T4_beg + y_T4_len;
    y_T5_len = ZeroBufferPoint;

    y_len = y_T1_len + y_T2_len + y_T3_len + 2 * BScanlines + y_T4_len + y_T5_len;

    if (pDataY)
        delete [] pDataY;
    pDataY = new unsigned short[y_len];

    for (unsigned int i = 0; i < y_len; ++i)
    {
        pDataY[i] = 0;
    }

    // y_T1_1 & x_T1_2 0V
    for (unsigned int i = 0 ; i < y_T1_1_len + y_T1_2_len; ++i)
    {
        pDataY[i + y_T1_beg] = Voltage_0V;
    }

    // y_T1_3: 0->-V CosTransit
    for (unsigned int i = 0; i < y_T1_3_len; ++i)
    {
        pDataY[i + y_T1_3_beg] = short(32768 + 65536.0 * Voltage * 0.5 /
            10000.0 * (cos(i * M_PI / (y_T1_3_len - 1)) - 1) + 0.5);
    }

    // y_T1_4: LinearScan -V->V
    for (int i = 0; i < y_T1_4_len; ++i)
    {
        pDataY[i + y_T1_4_beg] = short(32768 + Voltage * 65536.0 / 10000.0 *
            (2 * i - y_T1_4_len + 1) / (y_T1_4_len - 1) + 0.5);
    }

    // y_T1_5: V->0 CosTransit
    for (unsigned int i = 0; i < y_T1_5_len; ++i)
    {
        pDataY[i + y_T1_5_beg] = short(32768 + 65536.0 * Voltage * 0.5 /
            10000.0 * (cos(i * M_PI / (y_T1_5_len - 1)) + 1) + 0.5);
    }

    // y_T2 0V
    for (unsigned int i = 0; i < y_T2_len; ++i)
    {
        pDataY[i + y_T2_beg] = Voltage_0V;
    }

    // Y_T3 0->-V CosTransit
    for (unsigned int i = 0; i < y_T3_len; ++i)
    {
        pDataY[i + y_T3_beg] = short(32768 + 65536.0 * Voltage * 0.5 /
            10000.0 * (cos(i * M_PI / (y_T3_len - 1)) - 1) + 0.5);
    }

    // TBscans
    int BB = BScanlines;
    for (int i = 0; i < BScanlines; ++i)
    {
        pDataY[2 * i + y_TBscans_beg] = pDataY[2 * i + 1 + y_TBscans_beg] =
            short(32768 + Voltage * 65536.0 / 10000.0 *
            (2 * i - BB + 1) / (BB - 1) + 0.5);
    }

    // Y_T4 V->0 CosTransit
    for (unsigned int i = 0; i < y_T4_len; ++i)
    {
        pDataY[i + y_T4_beg] = short(32768 + 65536.0 * Voltage * 0.5 /
            10000.0 * (cos(i * M_PI / (y_T4_len - 1)) + 1) + 0.5);
    }

    // y_T5 0V
    for (unsigned int i = 0; i < y_T5_len; ++i)
    {
        pDataY[i + y_T5_beg] = Voltage_0V;
    }

    // 写入扫描信号到数据
    std::ofstream file2("scanY.txt");
    for (unsigned int i = 0; i < y_len; ++i)
    {
        file2 << pDataY[i] << std::endl;
    }
    file2.close();

    if (pSegmentInfoY)
        delete [] pSegmentInfoY;
    pSegmentInfoY = new USB3020_SEGMENT_INFO[6 + BScanlines];
    pSegmentInfoY[0].SegmentSize = y_T1_len;
    pSegmentInfoY[0].SegLoopCount = 1;
    pSegmentInfoY[1].SegmentSize = y_T2_len;
    pSegmentInfoY[1].SegLoopCount = 1;
    pSegmentInfoY[2].SegmentSize = y_T3_len;
    pSegmentInfoY[2].SegLoopCount = 1;
    for (unsigned int i = 0; i < BScanlines; ++i)
    {
        pSegmentInfoY[3 + i].SegmentSize = 2;
        pSegmentInfoY[3 + i].SegLoopCount = len_RampScan / 2 * AVERAGE_NUM;
    }
    pSegmentInfoY[3 + BScanlines].SegmentSize = y_T4_len;
    pSegmentInfoY[3 + BScanlines].SegLoopCount = 1;
    pSegmentInfoY[4 + BScanlines].SegmentSize = y_T5_len - 2;
    pSegmentInfoY[4 + BScanlines].SegLoopCount = 16777215;
    pSegmentInfoY[5 + BScanlines].SegmentSize = 2;
    pSegmentInfoY[5 + BScanlines].SegLoopCount = 1;

}

void DA_USB3020::GenDataT()
{
    //////////////////////////////////////////////////////////////////////////
    // T通道总共分为4段，每段的起始点和长度参考X通道
    //////////////////////////////////////////////////////////////////////////
    CosTransitPoint = long(SamplesPerSec * 0.5 * (1-duty_cycle) / Frequency + 0.5);
    if (pDataT)
        delete [] pDataT;
    pDataT = new unsigned short[x_T1_len + x_T2_len + x_T3_len + x_T4_len];

    for (unsigned int i = 0; i < x_T1_len + x_T2_len + x_T3_len + x_T4_len; ++i)
    {
        pDataT[i] = Voltage_0V;
    }

//	// T1
//	for (unsigned int i = 0; i < x_T1_2_len; ++i)
//	{
//		pDataT[i + x_T1_2_beg] = Voltage_5V;
//		//pDataT[i + x_T1_4_beg] = Voltage_0V;

//	}

//	for (unsigned int i = 0; i < x_T1_4_len; ++i)
//	{
//		pDataT[i + x_T1_4_beg] = Voltage_5V;
//	}


//	// T3
//	for (unsigned int i = 0; i < x_T3_2_len; ++i)
//	{
//		pDataT[i + x_T3_2_beg] = Voltage_5V;
//	}


    unsigned int delay1 = 82;
    // T1
    for (unsigned int i = delay1; i < x_T1_2_len; ++i)
    {
        pDataT[i + x_T1_2_beg] = Voltage_5V;
        //pDataT[i + x_T1_4_beg] = Voltage_0V;

    }

    for (unsigned int i = delay1; i < x_T1_4_len; ++i)
    {
        pDataT[i + x_T1_4_beg] = Voltage_5V;
    }

    unsigned int delay2 = 82;
    // T3
    for (unsigned int i = delay2; i < x_T3_2_len; ++i)
    {
        pDataT[i + x_T3_2_beg] = Voltage_5V;
    }

    // 写入扫描信号到数据
    std::ofstream file3("scanT.txt");
    for (unsigned int i = 0; i < x_T1_len + x_T2_len + x_T3_len + x_T4_len; ++i)
    {
        file3 << pDataT[i] << std::endl;
    }
    file3.close();

    if (pSegmentInfoT)
        delete [] pSegmentInfoT;
    pSegmentInfoT = new USB3020_SEGMENT_INFO[5];
    pSegmentInfoT[0].SegmentSize = x_T1_len;
    pSegmentInfoT[0].SegLoopCount = 1;
    pSegmentInfoT[1].SegmentSize = x_T2_len;
    pSegmentInfoT[1].SegLoopCount = CosTransitPoint / ZeroBufferPoint + 1;
    pSegmentInfoT[2].SegmentSize = x_T3_len;
    pSegmentInfoT[2].SegLoopCount = BScanlines * AVERAGE_NUM;
    pSegmentInfoT[3].SegmentSize = x_T4_len - 2;
    pSegmentInfoT[3].SegLoopCount = 16777215;
    pSegmentInfoT[4].SegmentSize = 2;
    pSegmentInfoT[4].SegLoopCount = 1;	// 无效

}

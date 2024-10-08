#include "DAcard.h"
#include <iostream>
#include <fstream>
using namespace std;

DA_USB3020::DA_USB3020()
{
    BScanPerSec = 20;
    BScanlines = 100;
	SamplesPerSec = 100000; ////////查一下原来是多少//////////
	ZeroBufferPoint = 10; // 0V电压的点数

    len_Total = int(SamplesPerSec * 1.0 / BScanPerSec + 0.5); 
    len_Transit = int(SamplesPerSec * 0.5 * (1-0.5) / BScanPerSec + 0.5); 
    len_Scan = len_Total - len_Transit * 2;

    pDataX = NULL;
    pDataY = NULL;
    pDataT = NULL;
    pSegmentInfoX = NULL;
    pSegmentInfoY = NULL;
    pSegmentInfoT = NULL;


    hDevice = USB3020_CreateDevice();
    DisableDA();
}

void DA_USB3020::CalculateDAdata()
{

    delete [] pDataX;
    delete [] pDataY;
    delete [] pDataT;
    pDataX = new unsigned short[len_Total + ZeroBufferPoint];
    pDataY = new unsigned short[BScanlines * AVERAGE_NUM];
    pDataT = new unsigned short[len_Total + ZeroBufferPoint];
    std::cout<<len_Scan<<std::endl;
    std::cout<<len_Transit<<std::endl;
    std::cout<<ZeroBufferPoint<<std::endl;
    for (int i = 0; i < len_Total + ZeroBufferPoint; i++)
    {
        pDataX[i] = 0;
        pDataT[i] = 0;
    }
    //  0->-V
    for (int i = 0; i < len_Transit; i++)
    {
        pDataX[i] = LinearCut(Voltage_0V, Voltage_5V_neg, len_Transit, i);
        pDataT[i] = Voltage_0V;

    }
    // -V->V
    for (int i = 0; i < len_Scan; i++)
    {
        pDataX[i + len_Transit] = LinearCut(Voltage_5V_neg, Voltage_5V, len_Scan, i);
        pDataT[i + len_Transit] = Voltage_5V;

    }
    // V->0
    for (int i = 0; i < len_Transit; i++)
    {
        pDataX[i + len_Transit + len_Scan] = LinearCut(Voltage_5V, Voltage_0V, len_Transit, i);
        pDataT[i + len_Transit + len_Scan] = Voltage_0V;

    }
    // 0V
    for (int i = 0 ; i < ZeroBufferPoint; i++)
    {
        pDataX[i + len_Total] = Voltage_0V;
        pDataT[i + len_Total] = Voltage_0V;

    }

    //std::cout<<pDataX<<std::endl;

    for (int i = 0; i < BScanlines * AVERAGE_NUM; i++)
    {
        int voltage = LinearCut(Voltage_5V_neg, Voltage_5V, BScanlines, i/AVERAGE_NUM);
        if (i % (2 * AVERAGE_NUM) == AVERAGE_NUM){
            voltage = LinearCut(Voltage_5V_neg, Voltage_5V, BScanlines, i/AVERAGE_NUM + 1);
        }
        if (i % (2 * AVERAGE_NUM) == AVERAGE_NUM + 1){
            voltage = LinearCut(Voltage_5V_neg, Voltage_5V, BScanlines, i/AVERAGE_NUM - 1);
        }
        for (int j = 0; j < len_Total + ZeroBufferPoint; j++){
            pDataY[i] = voltage;
        }

    }

    delete [] pSegmentInfoX;
    delete [] pSegmentInfoY;
    delete [] pSegmentInfoT;
    pSegmentInfoX = new USB3020_SEGMENT_INFO[1];
    pSegmentInfoY = new USB3020_SEGMENT_INFO[1];
    pSegmentInfoT = new USB3020_SEGMENT_INFO[1];

    pSegmentInfoX[0].SegmentSize = len_Total + ZeroBufferPoint;
    pSegmentInfoX[0].SegLoopCount = 1000000;
    pSegmentInfoT[0].SegmentSize = len_Total + ZeroBufferPoint;
    pSegmentInfoT[0].SegLoopCount = 1000000;
    pSegmentInfoY[0].SegmentSize = BScanlines * AVERAGE_NUM;
    pSegmentInfoY[0].SegLoopCount = 1;
}

bool DA_USB3020::InitDAForScan()
{
    PUSB3020_PARA_DA pPara = new USB3020_PARA_DA;
    pPara->OutputRange = USB3020_OUTPUT_N5000_P5000mV; 
    pPara->Frequency = SamplesPerSec;				
    pPara->LoopCount = 0;							
    pPara->TriggerSource = USB3020_TRIGSRC_SOFT_DA;	
    pPara->TriggerDir = USB3020_TRIGDIR_POSITIVE;	
    pPara->ClockSource = USB3020_CLOCKSRC_IN;		
    pPara->bSingleOut = false;
    pPara->TriggerMode = USB3020_TRIGMODE_CONTINUOUS;

    bool bstatus = USB3020_InitDeviceDA(hDevice, 1, pSegmentInfoX, pPara, 0); // DA0
    std::cout<<bstatus<<" init"<<std::endl;
    USB3020_InitDeviceDA(hDevice, 1, pSegmentInfoT, pPara, 1); // DA1

    pPara->Frequency = BScanPerSec;
    USB3020_InitDeviceDA(hDevice, 1, pSegmentInfoY, pPara, 2); // DA3

    long nRetSizeWords;
    nRetSizeWords = 0;
    bstatus = USB3020_WriteDeviceBulkDA(hDevice, pDataX, len_Total + ZeroBufferPoint, &nRetSizeWords, 0);
    std::cout<<bstatus<<" write"<<std::endl;
    nRetSizeWords = 0;
    USB3020_WriteDeviceBulkDA(hDevice, pDataT, len_Total + ZeroBufferPoint, &nRetSizeWords, 1 );

    nRetSizeWords = 0;
    USB3020_WriteDeviceBulkDA(hDevice, pDataY, BScanlines * AVERAGE_NUM, &nRetSizeWords, 2);
    return true;

}

bool DA_USB3020::EnableDA()
{
    USB3020_EnableDeviceDA(hDevice, 0);
    USB3020_EnableDeviceDA(hDevice, 1);
    bool bstatus = USB3020_EnableDeviceDA(hDevice, 2);
    std::cout<<bstatus<<" enable"<<std::endl;
    USB3020_SetDeviceTrigDA(hDevice, true, 0);
    bstatus = USB3020_SetDeviceTrigDA(hDevice, true, 1);
    std::cout<<bstatus<<" trig"<<std::endl;
    return true;
}

bool DA_USB3020::DisableDA()
{
    USB3020_DisableDeviceDA(hDevice, 0);
    bool bstatus = USB3020_DisableDeviceDA(hDevice, 1);
    USB3020_DisableDeviceDA(hDevice, 2);
    USB3020_DisableDeviceDA(hDevice, 3);
    std::cout<<bstatus<<" disable"<<std::endl;
    USB3020_ReleaseDeviceDA(hDevice, 0);
    bstatus = USB3020_ReleaseDeviceDA(hDevice, 1);
    USB3020_ReleaseDeviceDA(hDevice, 2);
    USB3020_ReleaseDeviceDA(hDevice, 3);   
    std::cout<<bstatus<<" release"<<std::endl; 
    return true;
}


DA_USB3020::~DA_USB3020()
{
    DisableDA();
    USB3020_ReleaseDevice(hDevice);   
    delete pSegmentInfoX;
    delete pSegmentInfoY;
    delete pSegmentInfoT;
    delete [] pDataX;
    delete [] pDataY;
    delete [] pDataT;

}

int DA_USB3020::LinearCut(int start, int end, int length, int i)
{  
    return int(start + (double)(end - start) / (length - 1) * i);
}


#include "DAcard.h"
#include <iostream>
#include <fstream>
#include "windows.h"
#include <conio.h>
using namespace std;

DA_USB3020::DA_USB3020()
{
    BScanPerSec = 1000;
    BScanlines = 100000;
	SamplesPerSec = 100000; 
	ZeroBufferPoint = 10; 

    len_Total = int(SamplesPerSec * 1.0 / BScanPerSec + 0.5); 
    len_Transit = int(SamplesPerSec * 0.5 * (1-0.9) / BScanPerSec + 0.5); 
    len_Scan = len_Total - len_Transit * 2;

    pDataX = NULL;
    pDataY = NULL;
    pDataT = NULL;

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
        std::cout<<LinearCut(Voltage_5V_neg, Voltage_5V, BScanlines, i/AVERAGE_NUM)<<std::endl;
        int voltage = LinearCut(Voltage_5V_neg, Voltage_5V, BScanlines, i/AVERAGE_NUM);
        /*if (i % (2 * AVERAGE_NUM) == AVERAGE_NUM){
            voltage = LinearCut(Voltage_5V_neg, Voltage_5V, BScanlines, i/AVERAGE_NUM + 1);
        }
        if (i % (2 * AVERAGE_NUM) == AVERAGE_NUM + 1){
            voltage = LinearCut(Voltage_5V_neg, Voltage_5V, BScanlines, i/AVERAGE_NUM - 1);
        }*/
        pDataY[i] = voltage;
    }

    memset(&SegmentInfoX, 0, sizeof(SegmentInfoX));
    memset(&SegmentInfoY, 0, sizeof(SegmentInfoY));
    memset(&SegmentInfoT, 0, sizeof(SegmentInfoT));


    SegmentInfoX[0].SegmentSize = len_Total + ZeroBufferPoint;
    SegmentInfoX[0].SegLoopCount = BScanlines * AVERAGE_NUM;
    SegmentInfoT[0].SegmentSize = len_Total + ZeroBufferPoint;
    SegmentInfoT[0].SegLoopCount = BScanlines * AVERAGE_NUM;
    SegmentInfoY[0].SegmentSize = BScanlines * AVERAGE_NUM;
    SegmentInfoY[0].SegLoopCount = 1;
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
    pPara->TriggerMode = USB3020_TRIGMODE_SINGLE;

    USB3020_InitDeviceDA(hDevice, 1, SegmentInfoX, pPara, 0); // DA0
    USB3020_InitDeviceDA(hDevice, 1, SegmentInfoT, pPara, 1); // DA1

    pPara->Frequency = BScanPerSec;
    USB3020_InitDeviceDA(hDevice, 1, SegmentInfoY, pPara, 2); // DA3

    long nRetSizeWords;
    nRetSizeWords = 0;
    USB3020_WriteDeviceBulkDA(hDevice, pDataX, len_Total + ZeroBufferPoint, &nRetSizeWords, 0);
    nRetSizeWords = 0;
    USB3020_WriteDeviceBulkDA(hDevice, pDataT, len_Total + ZeroBufferPoint, &nRetSizeWords, 1);
    nRetSizeWords = 0;
    USB3020_WriteDeviceBulkDA(hDevice, pDataY, BScanlines * AVERAGE_NUM, &nRetSizeWords, 2);
    return true;
}

bool DA_USB3020::EnableDA()
{
    USB3020_EnableDeviceDA(hDevice, 0);
    USB3020_EnableDeviceDA(hDevice, 1);
    USB3020_EnableDeviceDA(hDevice, 2);

    USB3020_SetDeviceTrigDA(hDevice, true, 0);
    USB3020_SetDeviceTrigDA(hDevice, true, 1);
    USB3020_SetDeviceTrigDA(hDevice, true, 2);
    


    while(!_kbhit()){
	    Sleep(10);
	    USB3020_GetDevStatusDA(hDevice, &DAStatus, 2);
	    printf("nCurSegNum=%2d  bconverting=%4x  ",  DAStatus.nCurSegNum, DAStatus.bConverting);
	    printf("nCurLoopCount=%4d  nCurSegLoopCount=%3d TR=%1d\n",  DAStatus.nCurLoopCount, DAStatus.nCurSegLoopCount, DAStatus.bTrigFlag);

    }

    return true;
}

bool DA_USB3020::DisableDA()
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

DA_USB3020::~DA_USB3020()
{
    DisableDA();
    USB3020_ReleaseDevice(hDevice);   
    delete [] pDataX;
    delete [] pDataY;
    delete [] pDataT;
}

int DA_USB3020::LinearCut(int start, int end, int length, int i)
{  
    return int(start + (double)(end - start) / (length - 1) * i);
}


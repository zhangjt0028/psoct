#include <cmath>        // std::abs
#include <iostream>
#include <vector>
#include <cstdint>
#include <DAcard.h>
#include <CaptureCard.h>
#include <DataProcess.h>
using U32 = uint32_t;
using U16 = uint16_t;

int main()
{
    //查询采集卡
    if (AlazarGetBoardBySystemID(1, 1))
    {
        std::cout<<"找到采集卡 "<<std::endl;
    }

    //连接DA卡和初始化相关的系数
    //SamplesPerSecond, ZeroBufferPoint, CosTransitPoint, XMode, YMode
    DA_USB3020* pDAUSB3020 = new DA_USB3020(200319L, 10, 100, 0, 2); 
    if (pDAUSB3020->isConnected())
    {
        std::cout<<"DA卡初始化成功"<<std::endl;
    }
    pDAUSB3020->DisableDA();
    //Voltage(mV), Frequency(Hz), duty_cycle, BScanlines(Frames), XMode, YMode
    pDAUSB3020->CalculateDAdata(1150, 225.4, 0.9, 800, 0, 2); 
    bool br = pDAUSB3020->WriteDataToDA();
    if (br)
    {
        std::cout<<"DA卡连接成功"<<std::endl;
    }

    int ScanMode = 22;

    StartAlazarADcapture(); //采集卡连接

    if(ScanMode == 22)//二维
    {
        pDAUSB3020->Start2DscanRepeat();
    }
    else if(ScanMode == 32)//三维
    {
        pDAUSB3020->Start3DscanRepeat();
    } ///DAStartScan

    

}

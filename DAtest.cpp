#include "DAcard.h"
#include <iostream>
#include <windows.h>
using namespace std;

int main(){
    
    
    DA_USB3020* pDAUSB3020 = new DA_USB3020(); 
    pDAUSB3020->CalculateDAdata(); 
    bool br = pDAUSB3020->InitDAForScan();
    if (br)
    {
        std::cout<<"success"<<std::endl;
    }
    pDAUSB3020->EnableDA();
    Sleep(10000);
    pDAUSB3020->DisableDA();
    std::cout<<"complete"<<std::endl;
    
}
/*
int LinearCut(unsigned int start, unsigned int end, unsigned long length, unsigned int i)
{

    if (i<10){
        std::cout<< start <<" "<< end <<" "<< length <<" "<< (double)(end - start) / (length - 1) << std::endl;
        
    }
    
    
    return int(start + (double)(end - start) / (length - 1) * i);
}
int main(){
    unsigned int BScanlines;
	unsigned int BScanPerSec;
	unsigned long SamplesPerSec; 
	unsigned long ZeroBufferPoint;
    
	int xscanMode;
    int yscanMode;

	unsigned long len_Transit;
	unsigned long len_Total;
	unsigned long len_Scan;
	unsigned long count;
    const static long Voltage_0V = 32768; 
    const static long Voltage_5V = 65535; 
    const static long Voltage_5V_neg = 0;
    std::cout<<Voltage_0V<< std::endl;
    BScanPerSec = 20;
    BScanlines = 100;
	SamplesPerSec = 1000000; ////////查一下原来是多少//////////
	ZeroBufferPoint = 10; // 0V电压的点数
    
    len_Total = long(SamplesPerSec * 1.0 / BScanPerSec + 0.5); 
    len_Transit = long(SamplesPerSec * 0.5 * (1-0.9) / BScanPerSec + 0.5); 
    len_Scan = len_Total - len_Transit * 2;

    unsigned short* pDataX;
    pDataX = new unsigned short[len_Total + ZeroBufferPoint];

    for (unsigned int i = 0; i < len_Transit; i++)
    {
        pDataX[i] = LinearCut(Voltage_0V, Voltage_5V_neg, len_Transit, i);

    }
}
*/

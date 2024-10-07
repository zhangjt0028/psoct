#include "DAcard.h"
#include <iostream>
#include <windows.h>
using namespace std;

int main(){
    
    
    DA_USB3020* pDAUSB3020 = new DA_USB3020(); 
    pDAUSB3020->CalculateDAdata(); 
    bool br = pDAUSB3020->InitDAForScan();
    std::cout<<"hello"<<std::endl;
    if (br)
    {
        std::cout<<"DA卡连接成功"<<std::endl;
    }
    Sleep(3000);
    pDAUSB3020->EnableDA();
    Sleep(100000);
    pDAUSB3020->DisableDA();
    
}
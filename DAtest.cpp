#include "DAcard.h"
#include <iostream>
#include <windows.h>
using namespace std;

int main(){
    
    DA_USB3020* pDAUSB3020 = new DA_USB3020(); 
    pDAUSB3020->CalculateDAdata(); 
    pDAUSB3020->InitDAForScan();
    pDAUSB3020->EnableDA();
    pDAUSB3020->DisableDA();
    delete pDAUSB3020;
}


#include<DataProcess.h>

DataProcess::DataProcess(){
    MAX_NUM_SAVED_VOLUMEM_IN_MEMORY  = 800;
    MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D = 3200;
    m_sampleLength = 1152;
    captureflag = 1;//0表示该状态不进行采集
    scanMode = 2; //2 or 3


}
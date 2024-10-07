#include "AlazarError.h"
#include "AlazarApi.h"
#include "AlazarCmd.h"
#include<iostream>
#include<vector>
#include<cstdint>
#include<mutex>
using U32 = uint32_t;
using U16 = uint16_t;

class DataCapture{
    public:
        DataCapture();
        static HANDLE           m_AlazarBoardHandle;
        std::vector<U16*>	    m_volumeMemBuffer;		// 整体缓存1s数据，大小6.48G
        U32						m_bytesPerBuffer;		// 一个BUFFer占用内存240M
        int						m_curIndexInMemBuffer;	// 代表最新的正在采集的数据的指针地址
        U32						m_buffersCompleted;		// 完成的采集的体积数量
        U32						m_timeout_ms;			// 等待buffer完整的时间上限
        int						m_lastAvailabeIndex;	// 最新的可用要被处理的内存的指针

        U32						m_recordperbuffer;		
        U32						m_samplesPerRecord;		
        U32						m_samplesPerBuffer;		
        U32						m_recordsPerAcquisition;// 0x7FFFFFFF 一直采集
        U32						m_admaFlags;			// ADMA_EXTERNAL_STARTCAPTURE | ADMA_NPT

        int						bufferCompletedLastTime;
        float					VolumePerSec;
        std::mutex              mutex1;
        static int BUFFER_COUNT = 4;
        static int MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D = 3200;
        void EnableCC();
};
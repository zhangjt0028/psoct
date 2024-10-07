#include "CaptureCard.h"

DataCapture::DataCapture(){
    
    m_AlazarBoardHandle = AlazarGetBoardBySystemID(1, 1);

    // 指定时钟的采样率为250MHz，并选择外时钟
    double samplesPerSec = 250000000.0;
    AlazarSetCaptureClock(m_AlazarBoardHandle, EXTERNAL_CLOCK, SAMPLE_RATE_250MSPS, CLOCK_EDGE_RISING, 0);

    // 确定chnnal A 和 B的一些input要求
    AlazarInputControl(m_AlazarBoardHandle, CHANNEL_A, DC_COUPLING, INPUT_RANGE_PM_400_MV, IMPEDANCE_50_OHM);
    AlazarInputControl(m_AlazarBoardHandle, CHANNEL_B, DC_COUPLING, INPUT_RANGE_PM_400_MV, IMPEDANCE_50_OHM);

    // 确定触发模式，只使用external trigger
    // TriggerLevelCode = 128 + 127 * TriggerLevelVolts / InputRangeVolts
    ////////not done//////////
    AlazarSetTriggerOperation(m_AlazarBoardHandle, TRIG_ENGINE_OP_J, TRIG_ENGINE_J, TRIG_EXTERNAL, TRIGGER_SLOPE_POSITIVE, 180, TRIG_ENGINE_K, TRIG_DISABLE, TRIGGER_SLOPE_POSITIVE, 128);
    
    AlazarSetExternalTrigger(m_AlazarBoardHandle, DC_COUPLING, ETR_TTL);

    // trigger timeout设置，避免采集卡没有接收到触发；
    double triggerTimeout_sec = 10;
    U32 triggerTimeout_clocks = (U32)(triggerTimeout_sec / 10.e-6 + 0.5);
    AlazarSetTriggerTimeOut(m_AlazarBoardHandle, triggerTimeout_clocks);

    AlazarConfigureAuxIO(m_AlazarBoardHandle, AUX_IN_TRIGGER_ENABLE, TRIGGER_SLOPE_POSITIVE);

    // 准备开始采集，先清空之前的buffer内存
    for (U32 bufferIndex = 0; bufferIndex < m_volumeMemBuffer.size(); bufferIndex++)
    {
        if (m_volumeMemBuffer[bufferIndex] != NULL)
        {
            VirtualFree(m_volumeMemBuffer[bufferIndex], 0, MEM_RELEASE);
        }
    }
    m_volumeMemBuffer.clear();

    m_recordperbuffer = 800;			//一个bscan有800线
    m_samplesPerRecord = 625;		// 一个line有625点
    m_samplesPerBuffer = m_samplesPerRecord * m_recordperbuffer; //1个Bscan组成的sample数


    //得到通道信息，分配一个bscan（buffer）的内存到pbuf
    U8 bitsPerSample;
    U32 maxSamplesPerChannel;

    AlazarGetChannelInfo(m_AlazarBoardHandle, &maxSamplesPerChannel, &bitsPerSample);
    float bytesPerSample = (float) ((bitsPerSample + 7) / 8);
    m_bytesPerBuffer = (U32)(bytesPerSample * m_samplesPerBuffer * 1 + 0.5);
    m_recordsPerAcquisition = 0x7FFFFFFF;

    for (U32 bufferIndex = 0; bufferIndex < MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D; bufferIndex++)
    {
        U16* pbuf = (U16 *)VirtualAlloc(NULL, m_bytesPerBuffer, MEM_COMMIT, PAGE_READWRITE);
        m_volumeMemBuffer.push_back(pbuf);
    }

    //采集前定义采集模式
    //m_admaFlags = ADMA_CONTINUOUS_MODE | ADMA_FIFO_ONLY_STREAMING;
    m_admaFlags = ADMA_EXTERNAL_STARTCAPTURE | ADMA_NPT | ADMA_FIFO_ONLY_STREAMING;
    AlazarBeforeAsyncRead(m_AlazarBoardHandle, CHANNEL_A, 0, m_samplesPerRecord, m_recordperbuffer, m_recordsPerAcquisition, m_admaFlags);
    //AlazarBeforeAsyncRead(m_AlazarBoardHandle, CHANNEL_B, 0, m_samplesPerRecord, m_recordperbuffer, m_recordsPerAcquisition, m_admaFlags);


    for (U32 bufferIndex = 0; bufferIndex < BUFFER_COUNT; bufferIndex++)
    {
        AlazarPostAsyncBuffer(m_AlazarBoardHandle, m_volumeMemBuffer[bufferIndex], m_bytesPerBuffer);
    }
    m_curIndexInMemBuffer = 0;
    m_buffersCompleted = 0;
    m_timeout_ms = 10;
    m_lastAvailabeIndex = -1;

    bufferCompletedLastTime = 0;
    VolumePerSec = 0;

    return true;
}

void DataCapture::EnableCC(){
    RETURN_CODE retCode = AlazarStartCapture(m_AlazarBoardHandle);
    if (retCode == ApiSuccess)
    {
        for(;;)
        {

            if (m_buffersCompleted == 3200)
            {
                retCode = AlazarAbortAsyncRead(m_AlazarBoardHandle);
                return;
            }

            U16 *pBuffer = m_volumeMemBuffer[m_curIndexInMemBuffer];

            //监测采集卡是否已经采集到足够数据
            retCode = AlazarWaitAsyncBufferComplete(m_AlazarBoardHandle, pBuffer, m_timeout_ms);

            if (retCode != ApiSuccess)
            {
                retCode = AlazarAbortAsyncRead(m_AlazarBoardHandle);
                return;
            }

            if (retCode == ApiSuccess)
            {
                {
                std::lock_guard<std::mutex> lock(mutex1);
                m_lastAvailabeIndex = m_curIndexInMemBuffer;
                m_buffersCompleted++;
                m_curIndexInMemBuffer = m_buffersCompleted % MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D;
                }
                
                U16 index = ((m_lastAvailabeIndex + BUFFER_COUNT) % MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D);

                retCode = AlazarPostAsyncBuffer(m_AlazarBoardHandle, m_volumeMemBuffer[index], m_bytesPerBuffer);//

                if (retCode != ApiSuccess)
                {
                    retCode = AlazarAbortAsyncRead(m_AlazarBoardHandle);
                    return;
                }
            }
            else
            {
                retCode = AlazarAbortAsyncRead(m_AlazarBoardHandle);
                return;
            }

        }
    }
}
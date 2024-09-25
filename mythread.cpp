#include "mythread.h"
#include "mainwidget.h"
#include <QDateTime>

mythread::mythread(QObject *parent) : QObject(parent)
{

}

void mythread::entry()
{
    RETURN_CODE retCode = AlazarStartCapture(mainWidget::m_AlazarBoardHandle);
    if (retCode == ApiSuccess)
    {
        for(;;)
        {
            if(mainWidget::captureflag == 1)
            {
                if (mainWidget::scanMode == 3 && m_buffersCompleted == 800)
                {
                    retCode = AlazarAbortAsyncRead(mainWidget::m_AlazarBoardHandle);
                    return;
                }
                if (mainWidget::scanMode == 32 && m_buffersCompleted == 3200)
                {
                    retCode = AlazarAbortAsyncRead(mainWidget::m_AlazarBoardHandle);
                    return;
                }

                U16 *pBuffer = m_volumeMemBuffer[m_curIndexInMemBuffer];
                //qDebug()<<"henhao";
                //监测采集卡是否已经采集到足够数据
                 //qDebug()<<"time1 "<<QTime::currentTime();
                retCode = AlazarWaitAsyncBufferComplete(mainWidget::m_AlazarBoardHandle,
                    pBuffer, m_timeout_ms);
                //qDebug()<<"time2 "<<QTime::currentTime();
                //qDebug()<<m_timeout_ms;
                if (retCode != ApiSuccess)
                {
                    qDebug()<<("AlazarWaitAsyncBufferComplete failed " + QVariant(AlazarErrorToText(retCode)).toString());
                    AlazarAbortAsyncRead(mainWidget::m_AlazarBoardHandle);
                    mainWidget::captureflag = 0;
                }

                if (retCode == ApiSuccess)
                {
                    {
                    QMutexLocker lock(&mutex1);
                    m_lastAvailabeIndex = m_curIndexInMemBuffer;
                    m_buffersCompleted++;
                    m_curIndexInMemBuffer = m_buffersCompleted % mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D;
                    }


                   // qDebug()<< m_curIndexInMemBuffer;
                    U16 index = ((m_lastAvailabeIndex + mainWidget::BUFFER_COUNT) % mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D);
                    //qDebug()<< index;
                    retCode = AlazarPostAsyncBuffer(mainWidget::m_AlazarBoardHandle,
                                m_volumeMemBuffer[index],
                                m_bytesPerBuffer);//
//                    qDebug()<<"time3 "<<QTime::currentTime();
//                    qDebug()<<"volum is" << (m_volumeMemBuffer[m_lastAvailabeIndex][1000] >> 4);
                    if (retCode != ApiSuccess)
                    {
                        mainWidget::captureflag = 0;
                        qDebug() << ("Error: AlazarPostAsyncBuffer failed -- %s",
                            AlazarErrorToText(retCode));
                        qDebug() << ("往回post出现问题，当前完成buffer数 %d", m_buffersCompleted);
                        retCode = AlazarAbortAsyncRead(mainWidget::m_AlazarBoardHandle);
                        //b_restart_2D_capture = true;
                    }
                }
                else
                {
                    mainWidget::captureflag = 0;
                }

            }
            else
            {
                break;
            }
        }
        retCode = AlazarAbortAsyncRead(mainWidget::m_AlazarBoardHandle);

    }
}

bool mythread::StartAlazarADcapture()
{
    // 先不考虑振镜扫描，后续考虑振镜扫描时，可以加入一个if判断，多configAUXIO口
    // 设置采集卡参数
    RETURN_CODE retCode;
    m_channelMask = CHANNEL_A;
    //retCode = AlazarSetParameterUL(mainWidget::m_AlazarBoardHandle,m_channelMask,SET_ADC_MODE, ADC_MODE_DES);


    // 指定时钟的采样率为400MHz，并选择外时钟
    double samplesPerSec = 400000000.0; //1GHZ采样率
    retCode = AlazarSetCaptureClock(mainWidget::m_AlazarBoardHandle, EXTERNAL_CLOCK,
        SAMPLE_RATE_250MSPS, CLOCK_EDGE_RISING, 0);

    // 确定chnnal A 和 B的一些input要求
    retCode = AlazarInputControl(mainWidget::m_AlazarBoardHandle,
        CHANNEL_A,
        DC_COUPLING,
        INPUT_RANGE_PM_400_MV,
        IMPEDANCE_50_OHM);

    retCode = AlazarInputControl(mainWidget::m_AlazarBoardHandle,
        CHANNEL_B,
        DC_COUPLING,
        INPUT_RANGE_PM_400_MV,
        IMPEDANCE_50_OHM);

    // 确定触发模式，只使用external trigger
    retCode = AlazarSetTriggerOperation(mainWidget::m_AlazarBoardHandle,
        TRIG_ENGINE_OP_J,
        TRIG_ENGINE_J,
        TRIG_EXTERNAL,
        TRIGGER_SLOPE_POSITIVE,
        180, //TriggerLevelCode = 128 + 127 * TriggerLevelVolts / InputRangeVolts
        TRIG_ENGINE_K,
        TRIG_DISABLE,
        TRIGGER_SLOPE_POSITIVE,
        128);
    retCode = AlazarSetExternalTrigger(mainWidget::m_AlazarBoardHandle,
        DC_COUPLING,
        ETR_TTL);

    // 设置trigger delay，这里设置为60个sample
    //double triggerDelay_sec = 0;
    //U32 triggerDelay_samples = (U32)(triggerDelay_sec * samplesPerSec + 0.5);
    //U32 triggerDelay_samples = (U32)0;
    //retCode = AlazarSetTriggerDelay(mainWidget::m_AlazarBoardHandle, triggerDelay_samples);

    // trigger timeout设置，避免采集卡没有接收到触发；
    double triggerTimeout_sec = 10;
    U32 triggerTimeout_clocks = (U32)(triggerTimeout_sec / 10.e-6 + 0.5);

    retCode = AlazarSetTriggerTimeOut(mainWidget::m_AlazarBoardHandle, triggerTimeout_clocks);
    //retCode = AlazarConfigureAuxIO(mainWidget::m_AlazarBoardHandle, AUX_OUT_TRIGGER, 0);
    retCode = AlazarConfigureAuxIO(mainWidget::m_AlazarBoardHandle, AUX_IN_TRIGGER_ENABLE, TRIGGER_SLOPE_POSITIVE);
    if (retCode != ApiSuccess)
    {
        qDebug() << ("Error: AlazarConfigureAuxIO failed " , AlazarErrorToText(retCode));
        return FALSE;
    }

    // 准备开始采集，先清空之前的buffer内存
    for (U32 bufferIndex = 0; bufferIndex < m_volumeMemBuffer.size(); bufferIndex++)
    {
        if (m_volumeMemBuffer[bufferIndex] != NULL)
        {
            VirtualFree(m_volumeMemBuffer[bufferIndex], 0, MEM_RELEASE);
        }
    }
    m_volumeMemBuffer.clear();

    m_recordperbuffer = 800;			//一个bscan有100线
    m_samplesPerRecord = mainWidget::m_sampleLength;		// 一个line有多少数据2304
    m_samplesPerBuffer = m_samplesPerRecord * m_recordperbuffer; //1个Bscan组成的sample数：10*2304


    //得到通道信息，分配一个bscan（buffer）的内存到pbuf
    U8 bitsPerSample;
    U32 maxSamplesPerChannel;

    retCode = AlazarGetChannelInfo(mainWidget::m_AlazarBoardHandle,
        &maxSamplesPerChannel, &bitsPerSample);
    float bytesPerSample = (float) ((bitsPerSample + 7) / 8);
    m_bytesPerBuffer = (U32)(bytesPerSample * m_samplesPerBuffer * 1 + 0.5);
    m_recordsPerAcquisition = 0x7FFFFFFF;

    for (U32 bufferIndex = 0; bufferIndex < mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D; bufferIndex++)
    {
        U16* pbuf = (U16 *)VirtualAlloc(NULL, m_bytesPerBuffer, MEM_COMMIT, PAGE_READWRITE);
        if (pbuf == NULL)
        {
            qDebug() <<("内存错误，无法分配buffer序号： " + QString::number(bufferIndex));
            return false;
        }
        m_volumeMemBuffer.push_back(pbuf);
    }

    //采集前定义采集模式
    //m_admaFlags = ADMA_CONTINUOUS_MODE | ADMA_FIFO_ONLY_STREAMING;
    m_admaFlags = ADMA_EXTERNAL_STARTCAPTURE | ADMA_NPT | ADMA_FIFO_ONLY_STREAMING;
    retCode = AlazarBeforeAsyncRead(mainWidget::m_AlazarBoardHandle, m_channelMask,
        0, // Must be 0
        m_samplesPerRecord,
        m_recordperbuffer,
        m_recordsPerAcquisition,
        m_admaFlags);
    if (retCode != ApiSuccess)
    {
        qDebug() <<("Error: AlazarBeforeAsyncRead failed -- " + QVariant(AlazarErrorToText(retCode)).toString());
        return false;
    }

    for (U32 bufferIndex = 0; bufferIndex < mainWidget::BUFFER_COUNT; bufferIndex++/*, pBuffer += samplesPerBuffer*/)
    {
        retCode = AlazarPostAsyncBuffer(mainWidget::m_AlazarBoardHandle, m_volumeMemBuffer[bufferIndex], m_bytesPerBuffer);
        if (retCode != ApiSuccess)
        {
            qDebug() << ("Error: AlazarPostAsyncBuffer" + QString::number(bufferIndex) + "failed");
            return false;//需要在此处退出线程
        }
    }
    m_curIndexInMemBuffer = 0;
    m_buffersCompleted = 0;
    m_timeout_ms = 10;
    m_lastAvailabeIndex = -1;

    bufferCompletedLastTime = 0;
    VolumePerSec = 0;

    return true;
}


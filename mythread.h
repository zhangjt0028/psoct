#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QObject>
#include "qdebug.h"
#include "AlazarError.h"
#include "AlazarApi.h"
#include "AlazarCmd.h"
#include <QMutex>
#include <QMutexLocker>
#include <QThread>


class mythread : public QObject
{
    Q_OBJECT
public:
    explicit mythread(QObject *parent = nullptr);
    QMutex mutex1;

    std::vector<U16*>		m_volumeMemBuffer;		// 整体缓存1s数据，大小6.48G
    U32						m_bytesPerBuffer;		// 一个BUFFer占用内存240M
    int						m_curIndexInMemBuffer;	// 代表最新的正在采集的数据的指针地址
    U32						m_buffersCompleted;		// 完成的采集的体积数量
    U32						m_timeout_ms;			// 等待buffer完整的时间上限
    int						m_lastAvailabeIndex;	// 最新的可用要被处理的内存的指针

    U32						m_recordperbuffer;		// 一个体有多少个Bscan，600
    U32						m_samplesPerRecord;		// 一个Bscan有多少数据400*500
    U32						m_samplesPerBuffer;		// recordperbuffer * samplesPerRecord
    U32						m_channelMask;			// CHANNEL_A
    U32						m_recordsPerAcquisition;// 0x7FFFFFFF 一直采集
    U32						m_admaFlags;			// ADMA_EXTERNAL_STARTCAPTURE | ADMA_NPT

    int						bufferCompletedLastTime;
    float					VolumePerSec;


public slots:
    void entry();

    bool StartAlazarADcapture();


signals:

};

#endif // MYTHREAD_H

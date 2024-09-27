#include <cmath>        // std::abs
#include <iostream>
#include <vector>
#include <cstdint>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <DAcard.h>
#include <CaptureCard.h>
#include <DataProcess.h>
using U32 = uint32_t;
using U16 = uint16_t;

std::queue<int> dataQueue; //存储采集到的数据
std::mutex queueMutex; //保护数据队列的互斥锁
std::condition_variable dataCondition; //通知数据可用
std::atomic<bool> isRunning(true); //运行状态



int main()
{
    //查询采集卡
    if (AlazarGetBoardBySystemID(1, 1))
    {
        std::cout<<"找到采集卡 "<<std::endl;
    }

    //连接DA卡和初始化相关的系数
    DA_USB3020* pDAUSB3020 = new DA_USB3020(); 
    if (pDAUSB3020->isConnected())
    {
        std::cout<<"DA卡初始化成功"<<std::endl;
    }
    //pDAUSB3020->DisableDA();
    pDAUSB3020->CalculateDAdata(); 
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


void dataAcquisition() {
    // 初始化 DA 卡
    initializeDACard(); // 根据你的 DA 卡API进行初始化

    while (isRunning) {
        // 等待触发信号
        waitForTrigger(); // 根据你的 DA 卡API实现等待触发的逻辑

        int data;
        // 读取采集卡的数据
        data = readDataFromCard(); // 调用采集卡API获取数据
        
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            dataQueue.push(data);
        }
        dataCondition.notify_one(); // 通知处理线程有新数据
    }

    // 结束时清理资源
    cleanupDACard(); // 关闭 DA 卡或释放资源
}


// 数据处理线程
void dataProcessing() {
    int data;
    while (isRunning) {
        std::unique_lock<std::mutex> lock(queueMutex);
        dataCondition.wait(lock, [] { return !dataQueue.empty() || !isRunning; }); // 等待数据
        while (!dataQueue.empty()) {
            data = dataQueue.front();
            dataQueue.pop();
            lock.unlock(); // 解锁以允许其他线程访问
            // 处理数据
            std::cout << "Processing data: " << data << std::endl;
            lock.lock(); // 重新上锁
        }
    }
}

int main() {
    std::thread acquisitionThread(dataAcquisition);
    std::thread processingThread(dataProcessing);

    // 运行一段时间后停止
    std::this_thread::sleep_for(std::chrono::seconds(5));
    isRunning = false; // 设置标志以停止线程

    // 通知处理线程结束
    dataCondition.notify_all();

    acquisitionThread.join();
    processingThread.join();

    return 0;
}

std::mutex bufferMutex;
std::condition_variable bufferReady;
std::queue<std::vector<short>> dataQueue;
bool stopSaving = false;

// 数据采集线程
void dataAcquisition() {
    while (!stopSaving) {
        std::vector<short> data = acquireData(); // 模拟采集数据
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            dataQueue.push(data);
        }
        bufferReady.notify_one();  // 通知写入线程
    }
}

// 数据写入线程
void saveDataToDisk() {
    while (!stopSaving || !dataQueue.empty()) {
        std::unique_lock<std::mutex> lock(bufferMutex);
        bufferReady.wait(lock, [] { return !dataQueue.empty(); });

        // 从队列中取出数据并写入硬盘
        std::vector<short> data = dataQueue.front();
        dataQueue.pop();
        lock.unlock();

        // 写入硬盘
        std::ofstream outFile("output.bin", std::ios::binary | std::ios::app);
        outFile.write(reinterpret_cast<char*>(data.data()), data.size() * sizeof(short));
    }
}













#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <vector>

const int BUFFER_SIZE = 1024 * 1024;  // 例如，缓冲区大小为 1 MB
std::vector<short> buffer1(BUFFER_SIZE);  // 缓冲区1
std::vector<short> buffer2(BUFFER_SIZE);  // 缓冲区2
std::vector<short>* currentBuffer = &buffer1;  // 当前用于采集的数据缓冲区
std::vector<short>* saveBuffer = nullptr;  // 保存到硬盘的缓冲区
std::mutex bufferMutex;
std::condition_variable bufferReady;
bool isBufferReady = false;
bool stopSaving = false;

// 采集线程函数
void dataAcquisition() {
    while (!stopSaving) {
        {
            std::lock_guard<std::mutex> lock(bufferMutex);

            // 采集卡模拟数据采集
            for (int i = 0; i < BUFFER_SIZE; ++i) {
                (*currentBuffer)[i] = rand() % 1000;  // 生成一些假数据
            }

            // 切换缓冲区
            saveBuffer = currentBuffer;
            currentBuffer = (currentBuffer == &buffer1) ? &buffer2 : &buffer1;
            isBufferReady = true;
        }
        bufferReady.notify_one();  // 通知保存线程保存数据
    }
}

// 保存线程函数
void saveToDisk() {
    std::ofstream outputFile("output.bin", std::ios::binary);
    
    while (!stopSaving || isBufferReady) {
        std::unique_lock<std::mutex> lock(bufferMutex);
        bufferReady.wait(lock, [] { return isBufferReady; });

        // 保存缓冲区中的数据到硬盘
        if (saveBuffer != nullptr) {
            outputFile.write(reinterpret_cast<char*>(saveBuffer->data()), BUFFER_SIZE * sizeof(short));
            isBufferReady = false;
        }
    }
    
    outputFile.close();
}

int main() {
    // 创建两个线程
    std::thread acquisitionThread(dataAcquisition);
    std::thread savingThread(saveToDisk);

    // 模拟运行一段时间
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    // 停止采集和保存
    stopSaving = true;
    bufferReady.notify_one();

    // 等待线程结束
    acquisitionThread.join();
    savingThread.join();

    return 0;
}




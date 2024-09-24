#include "mainwidget.h"
#include "ui_mainwidget.h"
#include "qcolor.h"
#include <math.h>
#include <cmath>        // std::abs
#include <iostream>

HANDLE mainWidget::m_AlazarBoardHandle;

int mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY  = 800;
int mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D = 3200;
int mainWidget::m_sampleLength = 1152;
//int mainWidget::m_sampleLength = 1152;
int mainWidget::captureflag = 0;//0表示该状态不进行采集
int mainWidget::BUFFER_COUNT = 4;
int mainWidget::scanMode = 1;
int mainWidget::xMode = 0; //fast = 0, slow = 2
int mainWidget::yMode = 2;
bool mainWidget::Bflag = true;
bool mainWidget::Wflag = true;//初始为不加窗
bool mainWidget::segmoveflag = true;
bool mainWidget::coarsemoveflag = true;
float mainWidget::sumedge = 0;

using namespace cv;


mainWidget::mainWidget(QWidget *parent):
    QWidget(parent),
    ui(new Ui::mainWidget)
{
   
        // 初始化采集卡
        U32 boardCount = AlazarBoardsInSystemBySystemID(1);
        if (boardCount == 1)
        {
           std::cout<<"系统中找到采集卡 "<<std::endl;
        }
        if (boardCount == 0)
        {
            std::cout<<"系统中没有找到采集卡 "<<std::endl;
        }
        mainWidget::m_AlazarBoardHandle = AlazarGetBoardBySystemID(1, 1);
        if (!mainWidget::m_AlazarBoardHandle)
        {
            std::cout<<"系统中没有找到采集卡 "<<std::endl;
        }


        //添加数据
        //连接DA卡和初始化相关的系数
        pDAUSB3020 = new DA_USB3020(200319L, 10, 100, 0, 2);
        if (pDAUSB3020->isConnected())
        {
            Voltage = 1150; //mV 修改振镜电压
            Frequency = 225.4; // Hz
            duty_cycle = 0.9;
            BScanlines = 800; // Frames
            std::cout<<"DA卡初始化成功"<<std::endl;
        }
        else
        {
            std::cout<<"DA卡初始化失败"<<std::endl;
        }
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        pDAUSB3020->CalculateDAdata(Voltage, Frequency, duty_cycle, BScanlines,mainWidget::xMode, mainWidget::yMode);
        bool br = pDAUSB3020->WriteDataToDA();
        if (br)
        {
            std::cout<<"DA卡连接成功"<<std::endl;
        }
        else
        {
            std::cout<<"DA卡连接失败"<<std::endl;
        }

        //初始化本底
        //m_BG = float[2048];
        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
       {
           m_BG[i] = 0;
       }
        //初始化hamming
        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
       {
           hammingwindow[i] = 1;
       }
       //memset(m_BG, 0, sizeof(float) * mainWidget::m_sampleLength);


        mainWidget::captureflag = 1;
        OCTplots();



}


mainWidget::~mainWidget()
{
    mainWidget::captureflag = 0;
    // 关闭DA卡程序
    pDAUSB3020->DisableDA();
    pDAUSB3020->ReleaseDA();
    pDAUSB3020->StopScan();

    firstThread->quit();
    firstThread->wait();

    killTimer(timerId1);
    killTimer(timerId2);
    killTimer(timerId3);
}


///线程///未解决///重要///
void mainWidget::on_connectBotton_clicked()
{
    firstThread->start();
    if (!ssoctThread->StartAlazarADcapture())
    {
        ui->textEdit->append("设置启动采集卡失败！");
        return;
    }
}


void mainWidget::timerEvent(QTimerEvent *event)
{
    if (event->timerId() == timerId1)
    {
        int curIndexInMEMbuffer = 0;
        int bufferCompleted;
        int availbleIndex = -1;
        {
        QMutexLocker lock(&ssoctThread->mutex1);
        bufferCompleted = ssoctThread->m_buffersCompleted;
        curIndexInMEMbuffer = ssoctThread->m_curIndexInMemBuffer;
        availbleIndex = ssoctThread->m_lastAvailabeIndex;
        }
        if (availbleIndex == -1)
        {
            return;
        }
        m_curDisplayData = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY];

        //availbleIndex = 1;
        //wxLogMessage("开始更新一维数据");
        {
        QMutexLocker lock(&ssoctThread->mutex1);

        memcpy(m_curDisplayData, ssoctThread->m_volumeMemBuffer[availbleIndex], mainWidget::m_sampleLength * 800 * sizeof(short));

        }



        float* datainfer = new float[mainWidget::m_sampleLength];

        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
        {
            datainfer[i] = ((m_curDisplayData[i] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
        }

        cosplot(datainfer);
        fftplot(datainfer);

        delete [] datainfer;
        delete [] m_curDisplayData;
    }

    if (event->timerId() == timerId2)
    {
        int curIndexInMEMbuffer = 0;
        int bufferCompleted;
        int availbleIndex = -1;

        {
        QMutexLocker lock(&ssoctThread->mutex1);
        bufferCompleted = ssoctThread->m_buffersCompleted;
        curIndexInMEMbuffer = ssoctThread->m_curIndexInMemBuffer;
        availbleIndex = ssoctThread->m_lastAvailabeIndex;

        }
        if (mainWidget::scanMode == 3  && bufferCompleted == 800)
        {
            pDAUSB3020->DisableDA();
            pDAUSB3020->ReleaseDA();
            pDAUSB3020->StopScan();
            return;
        }
        if (mainWidget::scanMode == 32 && bufferCompleted == 3200)
        {
            pDAUSB3020->DisableDA();
            pDAUSB3020->ReleaseDA();
            pDAUSB3020->StopScan();
            return;
        }
        if (availbleIndex == -1)
        {
            return;
        }

//        if (availbleIndex > 1)
//        {
        m_curDisplayData = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY];

//        {
//        QMutexLocker lock(&ssoctThread->mutex1);

        memcpy(m_curDisplayData, ssoctThread->m_volumeMemBuffer[availbleIndex], mainWidget::m_sampleLength * 800 * sizeof(short));

//        }
        p_F2 = new float[mainWidget::m_sampleLength * 800];
        for (int j = 0; j < 800; j++)
        {
            for (int i = 0; i < mainWidget::m_sampleLength; i++)
            {
                p_F2[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayData[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
            }
        }
        
        fftBscan(p_F2);

        delete [] m_curDisplayData;

        delete [] p_F2;


    }
    //二维血流
    if (event->timerId() == timerId3)
    {
        //qDebug()<<"time1 "<<QTime::currentTime();
        int curIndexInMEMbuffer = 0;
        int bufferCompleted;
        int availbleIndex = -1;
        {
        QMutexLocker lock(&ssoctThread->mutex1);
        bufferCompleted = ssoctThread->m_buffersCompleted;
        curIndexInMEMbuffer = ssoctThread->m_curIndexInMemBuffer;
        availbleIndex = ssoctThread->m_lastAvailabeIndex;
        }
        if (availbleIndex == -1)
        {
            return;
        }
        //if(mainWidget::scanMode == 22 | mainWidget::scanMode == 32)
        if(mainWidget::scanMode == 22)// | mainWidget::scanMode == 32)
        {
            if (availbleIndex > 2)
            {
                m_curDisplayDataAngio1 = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY];

                m_curDisplayDataAngio2 = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY];

                {

                QMutexLocker lock(&ssoctThread->mutex1);

                memcpy(m_curDisplayDataAngio1, ssoctThread->m_volumeMemBuffer[availbleIndex], mainWidget::m_sampleLength * 800 * sizeof(short));

                memcpy(m_curDisplayDataAngio2, ssoctThread->m_volumeMemBuffer[availbleIndex - 2], mainWidget::m_sampleLength * 800 * sizeof(short));

                }

                p_A1 = new float[mainWidget::m_sampleLength * 800];
                p_A2 = new float[mainWidget::m_sampleLength * 800];
                p_A3 = new float[mainWidget::m_sampleLength * 1600];
                for (int j = 0; j < 800; j++)
                {
                    for (int i = 0; i < mainWidget::m_sampleLength; i++)
                    {
                        p_A1[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayDataAngio1[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
                        p_A2[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayDataAngio2[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
                    }
                }
                memcpy(p_A3, p_A1, sizeof(float) * mainWidget::m_sampleLength * 800);

                memcpy(p_A3 + mainWidget::m_sampleLength * 800, p_A2, sizeof(float) * mainWidget::m_sampleLength * 800);
                //fftBscan(p_A1);
                fftBscanAngio2(p_A3);
                delete [] m_curDisplayDataAngio1;
                delete [] m_curDisplayDataAngio2;
                delete [] p_A1;
                delete [] p_A2;
                delete [] p_A3;
            }
        }



        if(mainWidget::scanMode == 10)
        {
            if (availbleIndex > 2)
            {
                m_curDisplayDataAngio1 = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY];

                m_curDisplayDataAngio2 = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY];

                if(availbleIndex % 2 == 1)
                {

                QMutexLocker lock(&ssoctThread->mutex1);

                memcpy(m_curDisplayDataAngio1, ssoctThread->m_volumeMemBuffer[availbleIndex], mainWidget::m_sampleLength * 800 * sizeof(short));

                memcpy(m_curDisplayDataAngio2, ssoctThread->m_volumeMemBuffer[availbleIndex - 1], mainWidget::m_sampleLength * 800 * sizeof(short));

                }
                if(availbleIndex % 2 == 0)
                {

                QMutexLocker lock(&ssoctThread->mutex1);

                memcpy(m_curDisplayDataAngio2, ssoctThread->m_volumeMemBuffer[availbleIndex], mainWidget::m_sampleLength * 800 * sizeof(short));

                memcpy(m_curDisplayDataAngio1, ssoctThread->m_volumeMemBuffer[availbleIndex - 1], mainWidget::m_sampleLength * 800 * sizeof(short));

                }

                p_A1 = new float[mainWidget::m_sampleLength * 800];
                p_A2 = new float[mainWidget::m_sampleLength * 800];
                p_A3 = new float[mainWidget::m_sampleLength * 1600];
                for (int j = 0; j < 800; j++)
                {
                    for (int i = 0; i < mainWidget::m_sampleLength; i++)
                    {
                        p_A1[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayDataAngio1[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
                        p_A2[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayDataAngio2[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
                    }
                }
                memcpy(p_A3, p_A1, sizeof(float) * mainWidget::m_sampleLength * 800);

                memcpy(p_A3 + mainWidget::m_sampleLength * 800, p_A2, sizeof(float) * mainWidget::m_sampleLength * 800);
                //fftBscan(p_A1);


            fftBscanCross(p_A3);

            qDebug()<<availbleIndex;
            //qDebug()<<"time2 "<<QTime::currentTime();
//            qDebug()<<"henhao" << p_A3[10 * mainWidget::m_sampleLength + 10] << p_A3[10 * mainWidget::m_sampleLength + 10 + mainWidget::m_sampleLength * 800];
//            qDebug()<<"henhao" << p_A1[10 * mainWidget::m_sampleLength + 10] << p_A2[10 * mainWidget::m_sampleLength + 10];
            delete [] p_A1;
            delete [] p_A2;
            delete [] p_A3;
            delete [] m_curDisplayDataAngio1;
            delete [] m_curDisplayDataAngio2;
            }
        }




    }


}

void mainWidget::cosplot(float* y_in)
{
    //定义两个可变数组存放绘图的坐标数据
    //QVector<double> x(2048),y_in(2048);//分别存放x和y坐标的数据,2049为数据长度


    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
    QVector<double> x_o(mainWidget::m_sampleLength);
    QVector<double> y_o(mainWidget::m_sampleLength);

    for (int i = 0; i < mainWidget::m_sampleLength; i++)
    {
        x_o[i] = i;
        y_o[i] = y_in[i];
    }

    ui->cosplot->graph(0)->setData(x_o,y_o);
    QColor color = QColor(133,238,255);
    ui->cosplot->graph(0)->setPen(QPen(color));

    ui->cosplot->replot();
//    qDebug()<<"pertime "<<QTime::currentTime();
}

// 用于计算出Bscan的fft
void mainWidget::fftBscan(float* numbers)
{
//    if (m_cosphy)
//            mkl_free(m_cosphy);
    m_cosphy = (float *)mkl_malloc(mainWidget::m_sampleLength * sizeof(float), 64);
//    if (m_sinphy)
//            mkl_free(m_sinphy);
    m_sinphy = (float *)mkl_malloc(mainWidget::m_sampleLength * sizeof(float), 64);


   
    
    /////////////////////////////////////
    //qDebug()<<"time1 "<<QTime::currentTime();

    int TEMP_BSCANS_LINES = 800;
    int TEMP_CSCANS_LINES = 1;

    const int numel = mainWidget::m_sampleLength * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES;



    MKL_Complex8* m_Cdata; // fft中的复数data
    m_Cdata = (MKL_Complex8*)mkl_malloc(mainWidget::m_sampleLength * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(MKL_Complex8), 32);

    MKL_Complex8* m_Cdata1; // fft中的复数data
    m_Cdata1 = (MKL_Complex8*)mkl_malloc(mainWidget::m_sampleLength * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(MKL_Complex8), 32);
    float tmp, phy;
    double m_dispersionW0 = ui->doubleSpinBoxw0->text().toDouble();//添加数据
    double m_dispersionA1 = ui->doubleSpinBoxa1->text().toDouble();//添加数据
    double m_dispersionA2 = ui->doubleSpinBoxa2->text().toDouble();//添加数据
    for (unsigned int i = 0; i < mainWidget::m_sampleLength; ++i)
    {
        tmp = (i - m_dispersionW0) * (i - m_dispersionW0);
        phy = m_dispersionA1 * tmp / 10000.0 + m_dispersionA2 * tmp * (i - m_dispersionW0) / 100000000.0;
        m_cosphy[i] = cos(phy);
        m_sinphy[i] = sin(phy);
    }
    
//    // 整块运算，对插值后的数据加入色散补偿，存入m_Cdata中。
//    for (unsigned int j = 0; j < 1; ++j)
//    {
//        for (unsigned int i = 0; i < mainWidget::m_sampleLength; ++i)
//        {
//            m_Cdata[j * mainWidget::m_sampleLength + i].real =
//                numbers[j * mainWidget::m_sampleLength + i] * m_cosphy[i];
//            m_Cdata[j * mainWidget::m_sampleLength + i].imag =
//                numbers[j * mainWidget::m_sampleLength + i] * m_sinphy[i];
//        }
//    }


    //qDebug()<<"m_cosphy"<<m_cosphy[50];
    //CalcDispersionParameter();
    for (unsigned int j = 0; j < TEMP_BSCANS_LINES; ++j)
    {
        for (unsigned int i = 0; i < mainWidget::m_sampleLength; ++i)
        {
            m_Cdata[j * mainWidget::m_sampleLength + i].real =
                numbers[j * mainWidget::m_sampleLength + i] * m_cosphy[i];
            m_Cdata[j * mainWidget::m_sampleLength + i].imag =
                numbers[j * mainWidget::m_sampleLength + i] * m_sinphy[i];
        }
    }

    
 
    
//    // 初始化mkl的fft命令;
    MKL_LONG status;
    DFTI_DESCRIPTOR_HANDLE m_FFThandle; // FFT handle

    status = DftiCreateDescriptor(&m_FFThandle, DFTI_SINGLE,
        DFTI_COMPLEX, 1, mainWidget::m_sampleLength);
    status = DftiSetValue(m_FFThandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    status = DftiSetValue(m_FFThandle, DFTI_NUMBER_OF_TRANSFORMS, TEMP_BSCANS_LINES * TEMP_CSCANS_LINES);
    status = DftiSetValue(m_FFThandle, DFTI_INPUT_DISTANCE, mainWidget::m_sampleLength);
    status = DftiSetValue(m_FFThandle, DFTI_OUTPUT_DISTANCE, mainWidget::m_sampleLength);
    status = DftiCommitDescriptor(m_FFThandle);
//    status = DftiCreateDescriptor(&m_FFThandle, DFTI_SINGLE,
//        DFTI_COMPLEX, 1, mainWidget::m_sampleLength);
//    status = DftiSetValue(m_FFThandle, DFTI_NUMBER_OF_TRANSFORMS, 1);
//    status = DftiSetValue(m_FFThandle, DFTI_INPUT_DISTANCE, mainWidget::m_sampleLength);
//    status = DftiCommitDescriptor(m_FFThandle);

    // 整块FFT
    DftiComputeForward(m_FFThandle, m_Cdata, m_Cdata1);

    status = DftiFreeDescriptor(&m_FFThandle);

    // 整块取模，结果保存在numbers中
    vcAbs(mainWidget::m_sampleLength * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES, m_Cdata1, numbers);


    float* numberslog;
    numberslog = (float*)mkl_malloc(mainWidget::m_sampleLength * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(float), 32);


    vslog10(&numel, numbers, numberslog);

    float alpha = 20;
    const int incx = 1;
    cblas_sscal(numel, alpha, numberslog, incx);

    mkl_free(m_Cdata);
    mkl_free(m_Cdata1);
    //mkl_free(numbers);

    // 开始考虑绘图
    plotBscan(numberslog);

    mkl_free(numberslog);
    mkl_free(m_cosphy);
    mkl_free(m_sinphy);
////    //qDebug()<<"time2 "<<QTime::currentTime();
}

void mainWidget::fftplot(float* p_F)
{
    m_cosphy = (float *)mkl_malloc(mainWidget::m_sampleLength * sizeof(float), 64);
    m_sinphy = (float *)mkl_malloc(mainWidget::m_sampleLength * sizeof(float), 64);
    float tmp, phy;
    double m_dispersionW0 = ui->doubleSpinBoxw0->text().toDouble();//添加数据
    double m_dispersionA1 = ui->doubleSpinBoxa1->text().toDouble();//添加数据
    double m_dispersionA2 = ui->doubleSpinBoxa2->text().toDouble();//添加数据
    for (unsigned int i = 0; i < mainWidget::m_sampleLength; ++i)
    {
        tmp = (i - m_dispersionW0) * (i - m_dispersionW0);
        phy = m_dispersionA1 * tmp / 10000.0 + m_dispersionA2 * tmp * (i - m_dispersionW0) / 100000000.0;
        m_cosphy[i] = cos(phy);
        m_sinphy[i] = sin(phy);
    }


     //qDebug()<<"time1 "<<QTime::currentTime();
    QVector<double> x_fft(mainWidget::m_sampleLength);
    QVector<double> y_fft(mainWidget::m_sampleLength);
//    QVector<double> x_fft1(mainWidget::m_sampleLength);
//    QVector<double> y_fft1(mainWidget::m_sampleLength);



    MKL_Complex8* tmp_yDisp ; // fft中的复数data
    tmp_yDisp = (MKL_Complex8*)mkl_malloc(mainWidget::m_sampleLength * sizeof(MKL_Complex8), 32);
    for (unsigned int i = 0; i < mainWidget::m_sampleLength; ++i)
    {
        tmp_yDisp[i].real = p_F[i] * m_cosphy[i];
        tmp_yDisp[i].imag = p_F[i] * m_sinphy[i];
    }

//    MKL_Complex8* tmp_y ; // fft中的复数data
//    tmp_y = (MKL_Complex8*)mkl_malloc(mainWidget::m_sampleLength * sizeof(MKL_Complex8), 32);
//    for (unsigned int i = 0; i < mainWidget::m_sampleLength; ++i)
//    {
//        tmp_y[i].real = p_F[i];
//        tmp_y[i].imag = 0;
//    }

    MKL_Complex8* tmp_y1Disp; // fft中的复数data
    tmp_y1Disp = (MKL_Complex8*)mkl_malloc(mainWidget::m_sampleLength * sizeof(MKL_Complex8), 32);
//    MKL_Complex8* tmp_y1; // fft中的复数data
//    tmp_y1 = (MKL_Complex8*)mkl_malloc(mainWidget::m_sampleLength * sizeof(MKL_Complex8), 32);
    // 初始化mkl的fft命令;
    MKL_LONG status;
    DFTI_DESCRIPTOR_HANDLE m_FFThandle; // FFT handle

//    status = DftiCreateDescriptor(&m_FFThandle, DFTI_SINGLE,
//        DFTI_COMPLEX, 1, mainWidget::m_sampleLength);
//    status = DftiSetValue(m_FFThandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
//    status = DftiSetValue(m_FFThandle, DFTI_NUMBER_OF_TRANSFORMS, 1);
//    status = DftiSetValue(m_FFThandle, DFTI_INPUT_DISTANCE, mainWidget::m_sampleLength);
//    status = DftiSetValue(m_FFThandle, DFTI_OUTPUT_DISTANCE, mainWidget::m_sampleLength);
//    status = DftiCommitDescriptor(m_FFThandle);

//    //普通整块FFT
//    DftiComputeForward(m_FFThandle, tmp_y, tmp_y1);
//    status = DftiFreeDescriptor(&m_FFThandle);
//    float* y_outF1;
//    y_outF1 = (float*)mkl_malloc(mainWidget::m_sampleLength * sizeof(float), 32);
//    // 整块取模，结果保存在numbers中

//    vcAbs(mainWidget::m_sampleLength, tmp_y1, y_outF1);
//    vslog10(&numel, y_outF1, y_outF1);

    // 初始化mkl的fft命令;
    status = DftiCreateDescriptor(&m_FFThandle, DFTI_SINGLE,
        DFTI_COMPLEX, 1, mainWidget::m_sampleLength);
    status = DftiSetValue(m_FFThandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    status = DftiSetValue(m_FFThandle, DFTI_NUMBER_OF_TRANSFORMS, 1);
    status = DftiSetValue(m_FFThandle, DFTI_INPUT_DISTANCE, mainWidget::m_sampleLength);
    status = DftiSetValue(m_FFThandle, DFTI_OUTPUT_DISTANCE, mainWidget::m_sampleLength);
    status = DftiCommitDescriptor(m_FFThandle);
    // 加色散的整块FFT
    DftiComputeForward(m_FFThandle, tmp_yDisp, tmp_y1Disp);
    status = DftiFreeDescriptor(&m_FFThandle);
    float* y_outF;
    y_outF = (float*)mkl_malloc(mainWidget::m_sampleLength * sizeof(float), 32);
    // 整块取模，结果保存在numbers中
    vcAbs(mainWidget::m_sampleLength, tmp_y1Disp, y_outF);




    //添加两个游标
    double a;
    double b;
    a = ui->spinBox->text().toDouble();   //添加数据
    b = ui->spinBox_2->text().toDouble();   //添加数据
    QVector<double> line1_x(2);
    QVector<double> line1_y(2);
    line1_x[0] = a;
    line1_x[1] = a;
    line1_y[0] = 20;
    line1_y[1] = 120;
    QVector<double> line2_x(2);
    QVector<double> line2_y(2);
    line2_x[0] = b;
    line2_x[1] = b;
    line2_y[0] = 20;
    line2_y[1] = 120;




    if (b>a)
    {
        float maxF(0);
        for (int i = a; i < b; ++i)
        {
            if (y_outF[i] > maxF)
            {
                maxF = y_outF[i];
            }
        }
        //qDebug() << maxF;
        float sigF(0), meanF(0);
        for (int i = b; i < b + 100; ++i)
        {
            meanF += y_outF[i];
        }
        meanF /= 100;
        for (int i = b; i < b + 100; ++i)
        {
            sigF += (y_outF[i] - meanF) * (y_outF[i] - meanF);
        }
        sigF /= 100;
        sigF = sqrt(sigF);
        ui->SNR->setNum(20*log10(maxF / sigF));
    }

    int numel = mainWidget::m_sampleLength;
    vslog10(&numel, y_outF, y_outF);


    mkl_free(tmp_yDisp);
    mkl_free(tmp_y1Disp);
//    mkl_free(tmp_y);
//    mkl_free(tmp_y1);

    for (int i = 0; i < mainWidget::m_sampleLength/2; i++)//色散的
    {
        x_fft[i] = i;
        y_fft[i] = 20 * y_outF[i];
    }

//    for (int i = 0; i < mainWidget::m_sampleLength/2; i++)//没色散的
//    {
//        x_fft1[i] = i;
//        y_fft1[i] = 20 * y_outF1[i];
//    }






//     ui->fftplot->addGraph();
//     QColor color = QColor(133,238,255);
//     ui->fftplot->graph(0)->setPen(QPen(color));
//     ui->fftplot->graph(0)->setData(x_fft,y_fft);
//    //ui->fftplot->graph(1)->setData(x_fft,y_fft);

//    ui->fftplot->replot();
//    mkl_free(y_outF);
    ui->fftplot->graph(0)->setData(x_fft,y_fft);
    QColor color = QColor(133,238,255);
    ui->fftplot->graph(0)->setPen(QPen(color));



   ui->fftplot->replot();
   mkl_free(y_outF);
//    mkl_free(y_outF1);
    //qDebug()<<"time3 "<<QTime::currentTime();

}

void mainWidget::plotBscan(float* num)
{
    // 考虑log图像的显示；
    double tmp1;
    double tmp2;
    tmp1 = ui->spinBox_3->text().toDouble();   //添加数据
    tmp2 = ui->spinBox_4->text().toDouble();   //添加数据
    if (tmp2 > tmp1)
    {

        cv::Mat img(mainWidget::m_sampleLength, 800, CV_32F);

            for (int j = 0; j < 800; j++)
            {
                for (int i = 0; i < mainWidget::m_sampleLength; i++)
                {
                    if (num[j * mainWidget::m_sampleLength + i ] < tmp1)
                    {
                        img.at<float>(i, j) = 0;
                    }
                    else if (num[j * mainWidget::m_sampleLength + i ] > tmp2)
                    {
                        img.at<float>(i, j) = 1;
                    }
                    else
                    {
                        img.at<float>(i, j) = (num[j * mainWidget::m_sampleLength + i] - tmp1) / (tmp2 - tmp1);
                    }
                }
            }
            cv::Mat img2(mainWidget::m_sampleLength/2, 720, CV_32F);
            img2 = img(Range(1, mainWidget::m_sampleLength/2), Range(1, 720));

            img2 = img2 * 255;
            // 考虑血流成像的C++程序的写作；
            // 先不封装成函数
            Mat Temp;

            img2.convertTo(Temp, CV_8UC1);


            QImage Qtemp = putImage(Temp);
            ui->label2D->setPixmap(QPixmap::fromImage(Qtemp.scaled(ui->label2D->size())));
    }
    else
    {
        return;
    }
}

void mainWidget::plotBscanCross1(float * num)
{
    plotBscan(num);
    // 考虑log图像的显示；
    double tmp1;
    double tmp2;
    tmp1 = ui->spinBox_3->text().toDouble();   //添加数据
    tmp2 = ui->spinBox_4->text().toDouble();   //添加数据
    if (tmp2 > tmp1)
    {

        cv::Mat img(mainWidget::m_sampleLength, 800, CV_32F);

            for (int j = 0; j < 800; j++)
            {
                for (int i = 0; i < mainWidget::m_sampleLength; i++)
                {
                    if (num[j * mainWidget::m_sampleLength + i + mainWidget::m_sampleLength * 800] < tmp1)
                    {
                        img.at<float>(i, j) = 0;
                    }
                    else if (num[j * mainWidget::m_sampleLength + i + mainWidget::m_sampleLength * 800] > tmp2)
                    {
                        img.at<float>(i, j) = 1;
                    }
                    else
                    {
                        img.at<float>(i, j) = (num[j * mainWidget::m_sampleLength + i+ mainWidget::m_sampleLength * 800] - tmp1) / (tmp2 - tmp1);
                    }
                }
            }
            cv::Mat img2(mainWidget::m_sampleLength/2, 720, CV_32F);
            img2 = img(Range(1, mainWidget::m_sampleLength/2), Range(1, 720));

            img2 = img2 * 255;
            // 考虑血流成像的C++程序的写作；
            // 先不封装成函数
            Mat Temp;

            img2.convertTo(Temp, CV_8UC1);


            QImage Qtemp = putImage(Temp);
            ui->labelflow->setPixmap(QPixmap::fromImage(Qtemp.scaled(ui->labelflow->size())));
    }
    else
    {
        return;
    }

}

void mainWidget::OCTplots()
{
    if(mainWidget::scanMode == 1)//一维扫描
    {
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        pDAUSB3020->Start1Dscan();


        //m_thread->start();
        timerId1 = startTimer(60);
    }
    else if(mainWidget::scanMode == 2)//2维扫描
    {
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        //pDAUSB3020->Start2Dscan();
        pDAUSB3020->Start2DscanRepeat();

        timerId2 = startTimer(60);

    }
    else if(mainWidget::scanMode == 22)//二维血流扫描
    {
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        pDAUSB3020->Start2DscanRepeat();

        timerId3 = startTimer(700);
    }
    else if(mainWidget::scanMode == 10) //二维交叉扫描
    {
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        pDAUSB3020->Start2Dscan();
        timerId3 = startTimer(100);
       //pDAUSB3020->Start2DscanRepeat();
    }
    else if(mainWidget::scanMode == 3)//三维扫描
    {
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        pDAUSB3020->Start3Dscan();
        timerId2 = startTimer(100);
    }
    else if(mainWidget::scanMode == 32)//三维扫描
    {
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        pDAUSB3020->Start3DscanRepeat();
        timerId2 = startTimer(100);
    }


}


void mainWidget::saveclicked()
{
    //储存1D数据，mainWidget::m_sampleLength个
    if (mainWidget::scanMode == 1)
    {
        int curIndexInMEMbuffer = 0;
        int bufferCompleted;
        int availbleIndex = -1;
        {
        QMutexLocker lock(&ssoctThread->mutex1);
        bufferCompleted = ssoctThread->m_buffersCompleted;
        curIndexInMEMbuffer = ssoctThread->m_curIndexInMemBuffer;
        availbleIndex = ssoctThread->m_lastAvailabeIndex;
        }
        if (availbleIndex == -1)
        {
            return;
        }

        U16 *curData = ssoctThread->m_volumeMemBuffer[availbleIndex];

        float* datainfer2 = new float[mainWidget::m_sampleLength];

        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
        {
            datainfer2[i] = ((curData[i] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
        }




        QFileDialog textsave(this,"save");
        textsave.setAcceptMode(QFileDialog::AcceptSave); // 设置文件对话框为保存模式
        textsave.setOptions(QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);//只显示文件夹
                                                      //所以手册里就没有定义，如果你要使用Save的话就自行定义一下吧
        textsave.setFileMode(QFileDialog::AnyFile);
        textsave.setNameFilter(tr("Text files (*.txt);;Images (*.png *.xpm *.jpg);; 2D (*.2d);; 3D (*.3d)"));//设定文件格式

        textsave.setViewMode(QFileDialog::Detail);
        QStringList qt;
        if(textsave.exec())
        {
            qt = textsave.selectedFiles();
        }
        qDebug()<<qt.at(0);
        QFile file(qt.at(0));
        file.open(QFile::WriteOnly|QFile::Text);
        QTextStream ts(&file);
        //ts << p_F3[10];//读取TextEdit的 内容 之前有看见其他版本用text();
        //qDebug() << p_F3[10];
        for (int i = 0; i < mainWidget::m_sampleLength; i++)
        {
            ts << datainfer2[i] << " ";
        }
//        QString dirName = QFileDialog::getExistingDirectory(this, "打开目录", "e:\\temp");
//        QMessageBox::information(this, "打开目录", "您选择的目录是: " + dirName);
   }
    if (mainWidget::scanMode == 2)
    {
        int curIndexInMEMbuffer = 0;
        int bufferCompleted;
        int availbleIndex = -1;
        {
            QMutexLocker lock(&ssoctThread->mutex1);
            bufferCompleted = ssoctThread->m_buffersCompleted;
            curIndexInMEMbuffer = ssoctThread->m_curIndexInMemBuffer;
            availbleIndex = ssoctThread->m_lastAvailabeIndex;
        }
        if (availbleIndex == -1)
        {
            return;
        }

        if (availbleIndex > 2)
        {
            m_curDisplayDataAngio1 = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY];

            m_curDisplayDataAngio2 = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY];

            {

            QMutexLocker lock(&ssoctThread->mutex1);

            memcpy(m_curDisplayDataAngio1, ssoctThread->m_volumeMemBuffer[availbleIndex], mainWidget::m_sampleLength * 800 * sizeof(short));

            memcpy(m_curDisplayDataAngio2, ssoctThread->m_volumeMemBuffer[availbleIndex - 1], mainWidget::m_sampleLength * 800 * sizeof(short));

            }

            p_A1 = new float[mainWidget::m_sampleLength * 800];
            p_A2 = new float[mainWidget::m_sampleLength * 800];
            p_A3 = new float[mainWidget::m_sampleLength * 1600];
            for (int j = 0; j < 800; j++)
            {
                for (int i = 0; i < mainWidget::m_sampleLength; i++)
                {
                    p_A1[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayDataAngio1[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
                    p_A2[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayDataAngio2[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
                }
            }
            memcpy(p_A3, p_A1, sizeof(float) * mainWidget::m_sampleLength * 800);

            memcpy(p_A3 + mainWidget::m_sampleLength * 800, p_A2, sizeof(float) * mainWidget::m_sampleLength * 800);
        }

        QFileDialog textsave(this,"save");
        textsave.setAcceptMode(QFileDialog::AcceptSave); // 设置文件对话框为保存模式
        textsave.setOptions(QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);//只显示文件夹
                                                      //所以手册里就没有定义，如果你要使用Save的话就自行定义一下吧
        textsave.setFileMode(QFileDialog::AnyFile);
        textsave.setNameFilter(tr("2D (*.2d);; 3D (*.3d)"));//设定文件格式

        textsave.setViewMode(QFileDialog::Detail);
        QStringList qt;
        if(textsave.exec())
        {
            qt = textsave.selectedFiles();
        }
        qDebug()<<qt.at(0);
        std::ofstream file1;
        file1.open(qt.at(0).toStdWString(), std::ios::binary);

        //ts << p_F3[10];//读取TextEdit的 内容 之前有看见其他版本用text();
        //qDebug() << p_F3[10];
        file1.write((char*)p_A3, sizeof(float) *
                        mainWidget::m_sampleLength * 800 * 2);

        file1.close();
    }

    if (mainWidget::scanMode == 32)
    {

        QFileDialog textsave(this,"save");
        textsave.setAcceptMode(QFileDialog::AcceptSave); // 设置文件对话框为保存模式
        textsave.setOptions(QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);//只显示文件夹
                                                      //所以手册里就没有定义，如果你要使用Save的话就自行定义一下吧
        textsave.setFileMode(QFileDialog::AnyFile);
        textsave.setNameFilter(tr("2D (*.2d);; 3D (*.3d)"));//设定文件格式

        textsave.setViewMode(QFileDialog::Detail);
        QStringList qt;
        if(textsave.exec())
        {
            qt = textsave.selectedFiles();
        }
        qDebug()<<qt.at(0);
        std::ofstream file1;
        file1.open(qt.at(0).toStdWString(), std::ios::binary);
        m_curDisplayDataAngio1 = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * 800];

        int curIndexInMEMbuffer = 0;
        int bufferCompleted;
        int availbleIndex = -1;
        {
            QMutexLocker lock(&ssoctThread->mutex1);
            bufferCompleted = ssoctThread->m_buffersCompleted;
            curIndexInMEMbuffer = ssoctThread->m_curIndexInMemBuffer;
            availbleIndex = ssoctThread->m_lastAvailabeIndex;
        }
        U16 *curData = ssoctThread->m_volumeMemBuffer[0];
        U16 *pTemp1 = curData;

        for (int i = 0; i < 3200; i++)//ssoctThread->m_volumeMemBuffer.size()
        {
            U16 *curData = ssoctThread->m_volumeMemBuffer[i];
            file1.write((char*)curData, sizeof(short) *mainWidget::m_sampleLength * 800);
        }

//        file1.write((char*)p_A3, sizeof(float) *
//            mainWidget::m_sampleLength * 800 * 3);

        file1.close();
        ui->textEdit->append("储存完毕 ");
    }

    if (mainWidget::scanMode == 3)
    {

        QFileDialog textsave(this,"save");
        textsave.setAcceptMode(QFileDialog::AcceptSave); // 设置文件对话框为保存模式
        textsave.setOptions(QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);//只显示文件夹
                                                      //所以手册里就没有定义，如果你要使用Save的话就自行定义一下吧
        textsave.setFileMode(QFileDialog::AnyFile);
        textsave.setNameFilter(tr("2D (*.2d);; 3D (*.3d)"));//设定文件格式

        textsave.setViewMode(QFileDialog::Detail);
        QStringList qt;
        if(textsave.exec())
        {
            qt = textsave.selectedFiles();
        }
        qDebug()<<qt.at(0);
        std::ofstream file1;
        file1.open(qt.at(0).toStdWString(), std::ios::binary);
        m_curDisplayDataAngio1 = new unsigned short[sizeof(short) * mainWidget::m_sampleLength * 800];

//        p_A4 = new float[mainWidget::m_sampleLength * 800];
//        p_A3 = new float[mainWidget::m_sampleLength * 800 * 3];
//        for (int curI = 1; curI < 3; curI++)//ssoctThread->m_volumeMemBuffer.size()
//        {
//            {
//                QMutexLocker lock(&ssoctThread->mutex1);
//                memcpy(m_curDisplayDataAngio1, ssoctThread->m_volumeMemBuffer[curI], mainWidget::m_sampleLength * 800 * sizeof(short));
//            }
//            //U16* tmpU16 = ssoctThread->m_volumeMemBuffer[curI];
//            for (int j = 0; j < 800; j++)
//            {
//                for (int i = 0; i < mainWidget::m_sampleLength; i++)
//                {
//                    p_A4[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayDataAngio1[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
//                }
//            }
//            memcpy(p_A3+mainWidget::m_sampleLength*800*curI, p_A4, sizeof(float) * mainWidget::m_sampleLength * 800);

//            //U16* tmpU16 = ssoctThread->m_volumeMemBuffer[curI];
//        }
        int curIndexInMEMbuffer = 0;
        int bufferCompleted;
        int availbleIndex = -1;
        {
            QMutexLocker lock(&ssoctThread->mutex1);
            bufferCompleted = ssoctThread->m_buffersCompleted;
            curIndexInMEMbuffer = ssoctThread->m_curIndexInMemBuffer;
            availbleIndex = ssoctThread->m_lastAvailabeIndex;
        }
        U16 *curData = ssoctThread->m_volumeMemBuffer[0];
        U16 *pTemp1 = curData;

//        for (int i = 0; i < 800; i++, curData += (mainWidget::m_sampleLength*800))//ssoctThread->m_volumeMemBuffer.size()
//        {
//            file1.write((char*)curData, sizeof(short) *mainWidget::m_sampleLength * 800);
//        }

        for (int i = 0; i < 800; i++)//ssoctThread->m_volumeMemBuffer.size()
        {
            U16 *curData = ssoctThread->m_volumeMemBuffer[i];
            file1.write((char*)curData, sizeof(short) *mainWidget::m_sampleLength * 800);
        }

//        file1.write((char*)p_A3, sizeof(float) *
//            mainWidget::m_sampleLength * 800 * 3);

        file1.close();
        ui->textEdit->append("储存完毕 ");
    }
}

void mainWidget::on_background_clicked()
{
    if (mainWidget::Bflag == true)
    {
        ui->background->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/backgroung2.png);}");
        int curIndexInMEMbuffer = 0;
        int bufferCompleted;
        int availbleIndex = -1;
        {
        QMutexLocker lock(&ssoctThread->mutex1);
        bufferCompleted = ssoctThread->m_buffersCompleted;
        curIndexInMEMbuffer = ssoctThread->m_curIndexInMemBuffer;
        availbleIndex = ssoctThread->m_lastAvailabeIndex;
        }
        if (availbleIndex == -1)
        {
            return;
        }
        //availbleIndex = 1;
        //wxLogMessage("开始更新一维数据");
        U16 *curData = ssoctThread->m_volumeMemBuffer[availbleIndex];

        for (int j = 0; j < mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY; j++)
        {
            for (int i = 0; i < mainWidget::m_sampleLength; i++)
            {
                m_BG[i] += ((curData[j * mainWidget::m_sampleLength + i ] >> 4) - 2048);
            }
        }
        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
        {
          m_BG[i] /= mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY;
        }
        mainWidget::Bflag = false;
    }
    else if (mainWidget::Bflag == false)
    {
        ui->background->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/backgroung.png);}");
        //本底归零
        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
       {
           m_BG[i] = 0;
       }
        mainWidget::Bflag = true;
    }


}

void mainWidget::on_addWindow_clicked()
{
    if (mainWidget::Wflag == false)
    {
        ui->addWindow->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/addWindow.png);}");
        mainWidget::Wflag = true;
        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
       {
           hammingwindow[i] = 1;
       }

    }
    else if (mainWidget::Wflag == true)
    {
        ui->addWindow->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/removeWindow.png);}");
        mainWidget::Wflag = false;
        //载入hamming
//        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
//       {
//           hammingwindow[i] = 0.54-0.46*cos(2 * 3.1415926 * i / mainWidget::m_sampleLength);
//       }
        double turki[1152] = {0,0.00011919,0.00047672,0.0010724,0.001906,0.002977,0.004285,0.0058293,0.0076093,0.009624,0.011873,0.014354,0.017067,0.02001,0.023181,0.026581,0.030206,0.034054,0.038126,0.042417,0.046926,0.051652,0.056591,0.061742,0.067101,0.072667,0.078437,0.084407,0.090576,0.09694,0.1035,0.11024,0.11717,0.12429,0.13158,0.13905,0.14669,0.1545,0.16247,0.17061,0.1789,0.18734,0.19594,0.20467,0.21355,0.22257,0.23172,0.24099,0.25039,0.25991,0.26955,0.27929,0.28914,0.29909,0.30913,0.31927,0.32949,0.33979,0.35017,0.36062,0.37114,0.38172,0.39235,0.40304,0.41377,0.42455,0.43536,0.4462,0.45706,0.46795,0.47885,0.48977,0.50068,0.5116,0.52251,0.53341,0.5443,0.55516,0.566,0.5768,0.58757,0.5983,0.60898,0.61961,0.63018,0.64069,0.65113,0.6615,0.67179,0.682,0.69213,0.70216,0.7121,0.72193,0.73166,0.74128,0.75079,0.76017,0.76943,0.77856,0.78756,0.79643,0.80515,0.81372,0.82215,0.83042,0.83854,0.84649,0.85428,0.8619,0.86934,0.87661,0.8837,0.89061,0.89733,0.90387,0.91021,0.91635,0.9223,0.92804,0.93358,0.93891,0.94404,0.94895,0.95365,0.95813,0.9624,0.96644,0.97026,0.97386,0.97723,0.98037,0.98329,0.98597,0.98842,0.99064,0.99263,0.99438,0.99589,0.99717,0.99821,0.99902,0.99958,0.99991,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.99991,0.99958,0.99902,0.99821,0.99717,0.99589,0.99438,0.99263,0.99064,0.98842,0.98597,0.98329,0.98037,0.97723,0.97386,0.97026,0.96644,0.9624,0.95813,0.95365,0.94895,0.94404,0.93891,0.93358,0.92804,0.9223,0.91635,0.91021,0.90387,0.89733,0.89061,0.8837,0.87661,0.86934,0.8619,0.85428,0.84649,0.83854,0.83042,0.82215,0.81372,0.80515,0.79643,0.78756,0.77856,0.76943,0.76017,0.75079,0.74128,0.73166,0.72193,0.7121,0.70216,0.69213,0.682,0.67179,0.6615,0.65113,0.64069,0.63018,0.61961,0.60898,0.5983,0.58757,0.5768,0.566,0.55516,0.5443,0.53341,0.52251,0.5116,0.50068,0.48977,0.47885,0.46795,0.45706,0.4462,0.43536,0.42455,0.41377,0.40304,0.39235,0.38172,0.37114,0.36062,0.35017,0.33979,0.32949,0.31927,0.30913,0.29909,0.28914,0.27929,0.26955,0.25991,0.25039,0.24099,0.23172,0.22257,0.21355,0.20467,0.19594,0.18734,0.1789,0.17061,0.16247,0.1545,0.14669,0.13905,0.13158,0.12429,0.11717,0.11024,0.1035,0.09694,0.090576,0.084407,0.078437,0.072667,0.067101,0.061742,0.056591,0.051652,0.046926,0.042417,0.038126,0.034054,0.030206,0.026581,0.023181,0.02001,0.017067,0.014354,0.011873,0.009624,0.0076093,0.0058293,0.004285,0.002977,0.001906,0.0010724,0.00047672,0.00011919,0};
        for (int i = 0; i < mainWidget::m_sampleLength; ++i)
       {
           hammingwindow[i] = turki[i];
       }
    }

}

void mainWidget::fftBscanAngio2(float * numbers)
{
//    clock_t start,finish;
//    double Times;
//    start=clock();
    const int CCD_PIXEL_NUMBER = mainWidget::m_sampleLength;
    const int TEMP_BSCANS_LINES = 800;
    const int TEMP_CSCANS_LINES = 2;

    const int numel = CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES;


    MKL_Complex8* m_Cdata; // fft中的复数data
    m_Cdata = (MKL_Complex8*)mkl_malloc(CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(MKL_Complex8), 32);

    MKL_Complex8* m_Cdata1; // fft中的复数data
    m_Cdata1 = (MKL_Complex8*)mkl_malloc(CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(MKL_Complex8), 32);

    for (int i = 0; i < numel; i++)
    {
        m_Cdata[i].real = numbers[i];
        m_Cdata[i].imag = 0;
    }

    // 初始化mkl的fft命令;
    MKL_LONG status;
    DFTI_DESCRIPTOR_HANDLE m_FFThandle; // FFT handle

    status = DftiCreateDescriptor(&m_FFThandle, DFTI_SINGLE,
        DFTI_COMPLEX, 1, CCD_PIXEL_NUMBER);
    status = DftiSetValue(m_FFThandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    status = DftiSetValue(m_FFThandle, DFTI_NUMBER_OF_TRANSFORMS, TEMP_BSCANS_LINES * TEMP_CSCANS_LINES);
    status = DftiSetValue(m_FFThandle, DFTI_INPUT_DISTANCE, CCD_PIXEL_NUMBER);
    status = DftiSetValue(m_FFThandle, DFTI_OUTPUT_DISTANCE, CCD_PIXEL_NUMBER);
    status = DftiCommitDescriptor(m_FFThandle);

    // 整块FFT
    DftiComputeForward(m_FFThandle, m_Cdata, m_Cdata1);

    status = DftiFreeDescriptor(&m_FFThandle);

    // 整块取模，结果保存在numbers中
    vcAbs(CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES, m_Cdata1, numbers);


    float* numberslog;
    numberslog = (float*)mkl_malloc(CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(float), 32);


    vslog10(&numel, numbers, numberslog);

    //vslog10(&numel, numbers, numberslog);

    float alpha = 20;
    const int incx = 1;
    cblas_sscal(numel, alpha, numberslog, incx);

    mkl_free(m_Cdata);
    mkl_free(m_Cdata1);

    plotBscan(numberslog);


    mkl_free(numberslog);

    // delete [] numbers;
//    // 读取连续四帧的图像数据


//    // 读取连续四帧的图像数据
//    start=clock();

    cv::Mat imgTrans(mainWidget::m_sampleLength, 1600, CV_32F);
    for (int k = 0; k < 2; k++)
    {
        for (int j = 0; j < 800; j++)
        {
            for (int i = 0; i < mainWidget::m_sampleLength; i++)
            {
                imgTrans.at<float>(i, k * 800 + j) = numbers[j * mainWidget::m_sampleLength + i + mainWidget::m_sampleLength * 800 * k];
            }

        }
    }


    cv::Mat angioOutput = cv::Mat::zeros(mainWidget::m_sampleLength/2, 800, CV_32F);
    int Measure = 2;
    cv::Mat imgTrans1 = imgTrans(Range(0, mainWidget::m_sampleLength/2), Range(0,1600));
    angioCM(imgTrans1, angioOutput, Measure);

    angioOutput = angioOutput * 255;


    Mat Temp;

    angioOutput.convertTo(Temp, CV_8UC1);



    QImage Qtemp = putImage(Temp);
    ui->labelflow->setPixmap(QPixmap::fromImage(Qtemp.scaled(ui->labelflow->size())));


}

void mainWidget::fftBscanCross(float *numbers)
{
    const int CCD_PIXEL_NUMBER = mainWidget::m_sampleLength;
    const int TEMP_BSCANS_LINES = 800;
    const int TEMP_CSCANS_LINES = 2;

    const int numel = CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES;


    MKL_Complex8* m_Cdata; // fft中的复数data
    m_Cdata = (MKL_Complex8*)mkl_malloc(CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(MKL_Complex8), 32);

    MKL_Complex8* m_Cdata1; // fft中的复数data
    m_Cdata1 = (MKL_Complex8*)mkl_malloc(CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(MKL_Complex8), 32);

    for (int i = 0; i < numel; i++)
    {
        m_Cdata[i].real = numbers[i];
        m_Cdata[i].imag = 0;
    }

//    // 初始化mkl的fft命令;
//    MKL_LONG status;
//    DFTI_DESCRIPTOR_HANDLE m_FFThandle; // FFT handle

//    status = DftiCreateDescriptor(&m_FFThandle, DFTI_SINGLE,
//        DFTI_COMPLEX, 1, CCD_PIXEL_NUMBER);
//    status = DftiSetValue(m_FFThandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
//    status = DftiSetValue(m_FFThandle, DFTI_NUMBER_OF_TRANSFORMS, TEMP_BSCANS_LINES * TEMP_CSCANS_LINES);
//    status = DftiSetValue(m_FFThandle, DFTI_INPUT_DISTANCE, CCD_PIXEL_NUMBER);
//    //status = DftiSetValue(m_FFThandle, DFTI_OUTPUT_DISTANCE, CCD_PIXEL_NUMBER);
//    status = DftiCommitDescriptor(m_FFThandle);


    // FFT初始化
    MKL_LONG status;
    DFTI_DESCRIPTOR_HANDLE m_FFThandle; // FFT handle

    status = DftiCreateDescriptor(&m_FFThandle, DFTI_SINGLE,
        DFTI_COMPLEX, 1, CCD_PIXEL_NUMBER);
    status = DftiSetValue(m_FFThandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    status = DftiSetValue(m_FFThandle, DFTI_NUMBER_OF_TRANSFORMS, TEMP_BSCANS_LINES * TEMP_CSCANS_LINES);
    status = DftiSetValue(m_FFThandle, DFTI_INPUT_DISTANCE, CCD_PIXEL_NUMBER);
    status = DftiSetValue(m_FFThandle, DFTI_OUTPUT_DISTANCE, CCD_PIXEL_NUMBER);
    status = DftiCommitDescriptor(m_FFThandle);


    // 整块FFT
    DftiComputeForward(m_FFThandle, m_Cdata, m_Cdata1);

    status = DftiFreeDescriptor(&m_FFThandle);

    // 整块取模，结果保存在numbers中
    vcAbs(CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES, m_Cdata1, numbers);


    float* numberslog;
    numberslog = (float*)mkl_malloc(CCD_PIXEL_NUMBER * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(float), 32);


    vslog10(&numel, numbers, numberslog);

    //vslog10(&numel, numbers, numberslog);

    float alpha = 20;
    const int incx = 1;
    cblas_sscal(numel, alpha, numberslog, incx);

    mkl_free(m_Cdata);
    mkl_free(m_Cdata1);
    //plotBscan(numberslog);
    plotBscanCross1(numberslog);

    mkl_free(numberslog);

}

void mainWidget::angioCM(Mat &dataTrans, Mat &angioOutput, int Measure)
{
    // 该程序是仿照我写的angioCM的Matlab程序来写的
     //qDebug()<<"cm1 "<<QTime::currentTime();
    int nPairs = Measure - 1;
//    // 考虑dataTrans为2048 * 3200的数组；

//    // 首先确定original的size
    int nA = dataTrans.cols; // 列数
    int mA = dataTrans.rows; //行数

    Mat angioInput1 = dataTrans(Range::all(), Range(0, nA / Measure * (Measure - 1))); // 存在内存泄露
    Mat angioInput2 = dataTrans(Range::all(), Range(nA / Measure, nA));

    int xx;
    int yy;
    xx = ui->spinBox_5->text().toInt();   //添加数据
    yy = ui->spinBox_6->text().toInt();   //添加数据

    cv::Mat windowCM = cv::Mat::ones(xx, yy, CV_32F);
    cv::Mat angioOut1;
    //二维滤波的写法
    //qDebug()<<"filter1 "<<QTime::currentTime();
    cv::filter2D(angioInput1, angioOut1, -1, windowCM, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

    angioOut1 = angioOut1 / (xx * yy); // 18 = 6 * 3;
    cv::Mat angioOut2;
    cv::filter2D(angioInput2, angioOut2, -1, windowCM, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    angioOut2 = angioOut2 / (xx * yy);
     //qDebug()<<"filter2 "<<QTime::currentTime();
//    //
    cv::Mat cmNumerator = (angioInput1 - angioOut1).mul(angioInput2 - angioOut2);
    cv::Mat cmIntensityAvel = (angioInput1 - angioOut1).mul(angioInput1 - angioOut1);
    cv::Mat cmIntensityAve2 = (angioInput2 - angioOut2).mul(angioInput2 - angioOut2);

    //angioOutput = dataTrans(Range(0, mainWidget::m_sampleLength), Range(0,800));
    // 再进行滤波的操作
    Mat Numeratorsum;
    cv::filter2D(cmNumerator, Numeratorsum, -1, windowCM, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

    Mat intensityAvesum1;
    cv::filter2D(cmIntensityAvel, intensityAvesum1, -1, windowCM, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

    Mat intensityAvesum2;
    cv::filter2D(cmIntensityAve2, intensityAvesum2, -1, windowCM, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

    // 求和
    Mat allSum;
    Mat sqrtIn;
    cv::sqrt(intensityAvesum1.mul(intensityAvesum2), sqrtIn);
    allSum = Numeratorsum / sqrtIn;

    // 需要提前将angioOutput赋值为0;
    for (int iPair = 0; iPair < nPairs; iPair++)
    {
        angioOutput = allSum(Range::all(), Range(nA / Measure * iPair, nA / Measure * (iPair + 1))) + angioOutput;
    }

    // 做一些简单的处理
    angioOutput = angioOutput / nPairs;
    cv::abs(angioOutput);
    angioOutput = 1 - angioOutput;

    // 考虑其中数，如果为1，则把其值赋值为0，可能并没有用

//    for (int i = 0; i < nA / Measure; i++)
//    {
//        for (int j = 0; j < mA; j++)
//        {
//            if (angioOutput.at<float>(j, i) > 0.99)
//            {
//                angioOutput.at<float>(j, i) = 0;
//            }
//        }
//    }
    //qDebug()<<"cm2 "<<QTime::currentTime();

}

//void mainWidget::CalcDispersionParameter()
//{
//	float tmp, phy;
//    int m_dispersionW0 = ui->dispw0->text().toInt();//添加数据
//    double m_dispersionA1 = ui->doubleSpinBoxa1->text().toDouble();//添加数据
//    double m_dispersionA2 = ui->doubleSpinBoxa2->text().toDouble();//添加数据
//    for (unsigned int i = 0; i < mainWidget::m_sampleLength; ++i)
//    {
//        tmp = (i - m_dispersionW0) * (i - m_dispersionW0);
//        phy = m_dispersionA1 * tmp / 10000.0 + m_dispersionA2 * tmp * (i - m_dispersionW0) / 100000000.0;
//        m_cosphy[i] = cos(phy);
//        m_sinphy[i] = sin(phy);
//    }
//}



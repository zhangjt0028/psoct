#include "mainwidget.h"
#include <math.h>
#include <cmath>        // std::abs
#include <iostream>






void mainWidget::timerEvent(QTimerEvent *event)
{

    if (event->timerId() == timerId3)
    {
        if (bufferCompleted == 3200)
        {
            pDAUSB3020->DisableDA();

            pDAUSB3020->StopScan();
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
    if (event->timerId() == timerId2)
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


}






void mainWidget::background()
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



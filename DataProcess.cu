#include<DataProcess.h>
#include<cufft.h>

DataProcess::DataProcess(){
    MAX_NUM_SAVED_VOLUMEM_IN_MEMORY  = 800;
    MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D = 3200;
    m_sampleLength = 1152;
    captureflag = 1;//0表示该状态不进行采集
    scanMode = 2; //2 or 3

    //hammingwindow[i] = 0.54-0.46*cos(2 * 3.1415926 * i / mainWidget::m_sampleLength);

    double turki[1152] = {0,0.00011919,0.00047672,0.0010724,0.001906,0.002977,0.004285,0.0058293,0.0076093,0.009624,0.011873,0.014354,0.017067,0.02001,0.023181,0.026581,0.030206,0.034054,0.038126,0.042417,0.046926,0.051652,0.056591,0.061742,0.067101,0.072667,0.078437,0.084407,0.090576,0.09694,0.1035,0.11024,0.11717,0.12429,0.13158,0.13905,0.14669,0.1545,0.16247,0.17061,0.1789,0.18734,0.19594,0.20467,0.21355,0.22257,0.23172,0.24099,0.25039,0.25991,0.26955,0.27929,0.28914,0.29909,0.30913,0.31927,0.32949,0.33979,0.35017,0.36062,0.37114,0.38172,0.39235,0.40304,0.41377,0.42455,0.43536,0.4462,0.45706,0.46795,0.47885,0.48977,0.50068,0.5116,0.52251,0.53341,0.5443,0.55516,0.566,0.5768,0.58757,0.5983,0.60898,0.61961,0.63018,0.64069,0.65113,0.6615,0.67179,0.682,0.69213,0.70216,0.7121,0.72193,0.73166,0.74128,0.75079,0.76017,0.76943,0.77856,0.78756,0.79643,0.80515,0.81372,0.82215,0.83042,0.83854,0.84649,0.85428,0.8619,0.86934,0.87661,0.8837,0.89061,0.89733,0.90387,0.91021,0.91635,0.9223,0.92804,0.93358,0.93891,0.94404,0.94895,0.95365,0.95813,0.9624,0.96644,0.97026,0.97386,0.97723,0.98037,0.98329,0.98597,0.98842,0.99064,0.99263,0.99438,0.99589,0.99717,0.99821,0.99902,0.99958,0.99991,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.99991,0.99958,0.99902,0.99821,0.99717,0.99589,0.99438,0.99263,0.99064,0.98842,0.98597,0.98329,0.98037,0.97723,0.97386,0.97026,0.96644,0.9624,0.95813,0.95365,0.94895,0.94404,0.93891,0.93358,0.92804,0.9223,0.91635,0.91021,0.90387,0.89733,0.89061,0.8837,0.87661,0.86934,0.8619,0.85428,0.84649,0.83854,0.83042,0.82215,0.81372,0.80515,0.79643,0.78756,0.77856,0.76943,0.76017,0.75079,0.74128,0.73166,0.72193,0.7121,0.70216,0.69213,0.682,0.67179,0.6615,0.65113,0.64069,0.63018,0.61961,0.60898,0.5983,0.58757,0.5768,0.566,0.55516,0.5443,0.53341,0.52251,0.5116,0.50068,0.48977,0.47885,0.46795,0.45706,0.4462,0.43536,0.42455,0.41377,0.40304,0.39235,0.38172,0.37114,0.36062,0.35017,0.33979,0.32949,0.31927,0.30913,0.29909,0.28914,0.27929,0.26955,0.25991,0.25039,0.24099,0.23172,0.22257,0.21355,0.20467,0.19594,0.18734,0.1789,0.17061,0.16247,0.1545,0.14669,0.13905,0.13158,0.12429,0.11717,0.11024,0.1035,0.09694,0.090576,0.084407,0.078437,0.072667,0.067101,0.061742,0.056591,0.051652,0.046926,0.042417,0.038126,0.034054,0.030206,0.026581,0.023181,0.02001,0.017067,0.014354,0.011873,0.009624,0.0076093,0.0058293,0.004285,0.002977,0.001906,0.0010724,0.00047672,0.00011919,0};
    for (int i = 0; i < m_sampleLength; ++i)
    {
        hammingwindow[i] = turki[i];
    }

    //初始化本底///////////////
    U16 *curData = ssoctThread->m_volumeMemBuffer[availbleIndex];

    for (int j = 0; j < MAX_NUM_SAVED_VOLUMEM_IN_MEMORY; j++)
    {
        for (int i = 0; i < m_sampleLength; i++)
        {
            m_BG[i] += ((curData[j * m_sampleLength + i ] >> 4) - 2048);
        }
    }
    for (int i = 0; i < m_sampleLength; ++i)
    {
        m_BG[i] /= MAX_NUM_SAVED_VOLUMEM_IN_MEMORY;
    }
    /////////////////////////
}



void DataProcess::fftBscan(float* numbers)
{
//    if (m_cosphy)
//            mkl_free(m_cosphy);
    m_cosphy = (float *)mkl_malloc(m_sampleLength * sizeof(float), 64);
//    if (m_sinphy)
//            mkl_free(m_sinphy);
    m_sinphy = (float *)mkl_malloc(m_sampleLength * sizeof(float), 64);


   
    
    /////////////////////////////////////
    //qDebug()<<"time1 "<<QTime::currentTime();

    int TEMP_BSCANS_LINES = 800;
    int TEMP_CSCANS_LINES = 1;

    const int numel = m_sampleLength * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES;



    MKL_Complex8* m_Cdata; // fft中的复数data
    m_Cdata = (MKL_Complex8*)mkl_malloc(m_sampleLength * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
        * sizeof(MKL_Complex8), 32);

    MKL_Complex8* m_Cdata1; // fft中的复数data
    m_Cdata1 = (MKL_Complex8*)mkl_malloc(m_sampleLength * TEMP_BSCANS_LINES * TEMP_CSCANS_LINES
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
        for (unsigned int i = 0; i < m_sampleLength; ++i)
        {
            m_Cdata[j * m_sampleLength + i].real =
                numbers[j * m_sampleLength + i] * m_cosphy[i];
            m_Cdata[j * m_sampleLength + i].imag =
                numbers[j * m_sampleLength + i] * m_sinphy[i];
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
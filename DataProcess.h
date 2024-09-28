
class DataProcess{
    public:
        DataProcess();
        ~DataProcess();

        void timerEvent(QTimerEvent *event);

        static int captureflag;
        static int m_sampleLength;
        static int MAX_NUM_SAVED_VOLUMEM_IN_MEMORY;
        static int MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D;
        static int scanMode; //扫描模式1为一维，2为二维，22为二维重复，3为三维，33为三维重复

        int timerId2;
        int timerId3;
        float* m_dataIn; // 原始的频域数据
        unsigned short*				m_curDisplayData;

        // 血流用数据
        unsigned short*				m_curDisplayDataAngio1;
        unsigned short*				m_curDisplayDataAngio2;
        float* p_A1;
        float* p_A2;
        float* p_A3;
        float* p_A4;
        unsigned short*				m_curData;

        QTimer *timer;
        CoordCalibrateByJointAngleAndTool current_Coord;
        static ToolInEndDesc tool;

        float* p_F2; // 存事件2lock出来的数据，用于二维成像
        float  m_BG[1152];
        float  hammingwindow[1152];
        float* m_cosphy; 
        float* m_sinphy;
        MKL_Complex8* m_Cdata;

        QMutex mutex1;

        void OCTplots();
        void fftBscan(float*); // 用于计算Bscan的fft等操作
        void plotBscan(float*); // 用于绘制Bscan数据
        void fftBscanAngio2(float*); //用于计算血流Bscan的fft等操作
        void fftBscanCross(float * );
        void angioCM(Mat& dataTrans, Mat& angioOutput, int Measure);

        //void CalcDispersionParameter();


        void background();
        void save();
        void addWindow();



    private:

};



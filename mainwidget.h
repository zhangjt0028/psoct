#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <mkl.h>
#include "AlazarError.h"
#include "AlazarApi.h"
#include "AlazarCmd.h"
#include <QThread>
#include "mythread.h"
#include "robotthread.h"
#include "qdebug.h"
#include <QTimer>
#include <QMutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "DAcard.h"
#include "QColorDialog"   //颜色对话框
//#include "3DCamera.hpp"
#include <QList>
#include <QMessageBox>
#include "camerathread.h"
#include "AuboRobotMetaType.h"
#include "rsdef.h"

using namespace cv;

namespace Ui {
class mainWidget;
}

class mainWidget : public QWidget
{
    Q_OBJECT

public:
    explicit mainWidget(QWidget *parent = nullptr);
    ~mainWidget();
    float* m_MEMbufF;
    static HANDLE m_AlazarBoardHandle;
    QThread *firstThread;
    mythread *ssoctThread;
    QThread *secondThread;
    robotthread *rbThread;
    QThread *thirdThread;
    camerathread *MyCamThread;

    QTimer fps_timer;

    void PrintBoardInfo();
    void PrepareProcessing1D();
    void timerEvent(QTimerEvent *event);

    // config the alazar car
    bool StartAlazarADcapture();


    static int  captureflag;


    static int   m_sampleLength;
    static int MAX_NUM_SAVED_VOLUMEM_IN_MEMORY;
    static int MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D;
    static int BUFFER_COUNT;
    static int scanMode; //扫描模式1为一维，2为二维，22为二维重复，3为三维，33为三维重复
    static int xMode;
    static int yMode;
    static bool Bflag;//0为不减本底，1为减
    static bool Wflag;//false为不加窗，true为加窗

    unsigned int Voltage;
    double Frequency;
    double duty_cycle;
    unsigned int BScanlines;

    int timerId1;
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

   //3D相机相关
   static ERROR_CODE ret;
   static ERROR_CODE cstatus;//判断是否连接
   static cs::ICameraPtr camera;
   static float exposure;
   static float gain;
   static int rgbFlag;//rgb勾选状态
   static int depthFlag;//depth勾选状态
   static int rgbX;//传入rgbx坐标
   static int rgbY;//传入rgby坐标
   static int depthX;//变换得到的深度坐标
   static int depthY;
   static int minRange;
   static int maxRange;
   static int identifyFlag;//识别标志的状态
   static int labelcolor;//得到标志的颜色
   static bool segmoveflag;
   static float sumedge;
   static bool coarsemoveflag;
   static float z1;
   static float z2;
   static float z3;
   static float z4;
   static float I3;
   static float I4;
   static float zposition;
   static int moveloopidx;

   // 机械臂相关
   bool Robot_Login();
   bool logout(RSHD rshd);
   void toolBox();
   static RSHD pri_rshd_send;
   //static RSHD pri_rshd_recive;
   static int tooltype;
   static double x;
   static double y;
   static double z;
   static double rx;
   static double ry;
   static double rz;

   QTimer *timer;
   CoordCalibrateByJointAngleAndTool current_Coord;
   static ToolInEndDesc tool;

    float* p_F2; // 存事件2lock出来的数据，用于二维成像
    float  m_BG[1152];
    float hammingwindow[1152];
    float* m_cosphy; 
    float* m_sinphy;
    MKL_Complex8* m_Cdata;

    DA_USB3020* pDAUSB3020;     // DA卡的类

    QMutex mutex1;
    void cosplot(float*);
    void fftplot(float*);
    void OCTplots();
    void fftBscan(float*); // 用于计算Bscan的fft等操作
    void plotBscan(float*); // 用于绘制Bscan数据
    void fftBfcous(float*); // 用于计算Bscan 焦点的fft等操作
    // 用于绘制得到Bscan图像的表面（画出来？），计算到焦面的距离，并反馈给机械臂要移动的距离，先只动一下；
    void plotBfocus(float*);

    void fftBfcous2(float*); // 用于计算Bscan 焦点的fft等操作
    // 用于黄金分割自动变焦
    void plotBfocus2(float*);
    void plotBscanCross1(float*);
    void fftBscanAngio2(float*); //用于计算血流Bscan的fft等操作

    void fftBscanCross(float * );
    void angioCM(Mat& dataTrans, Mat& angioOutput, int Measure);
    QImage putImage(const Mat&); // 用于将Mat数据转换成QIMAGE格式的数据
    //void CalcDispersionParameter();


private slots:
    void on_startButton_clicked();

    void on_connectBotton_clicked();

    void on_stopButton_clicked();

    void on_comboBox_activated(const QString &arg1);

    //void on_DAtest_clicked();

    void on_resetaxis_clicked();

    void on_saveButton_clicked();

    void on_textEdit_textChanged();

    void on_background_clicked();

    //void hammingWindow(float);

    void saveclicked();
    void on_addWindow_clicked();

    void on_connectDA_clicked();

    void on_comboBox_2_activated(const QString &arg1);

    // 机械臂相关
//    void on_pb_login_clicked();
    void displayRobotData();

    void on_pb_login_clicked();

    void on_logoutButton_clicked();

    void on_saveData_clicked();

    void recieveIK(int);


    //三维相机相关
    void on_connectcameraButton_clicked();

    void on_opencameraButton_clicked();

    void receiveimage(cv::Mat);

    void receiveimage2(cv::Mat);

    void receiveimage3(cv::Mat, cv::Mat);
    void on_closecameraButton_clicked();

    void on_identifylabel_clicked();

    void on_closeidentify_clicked();


    void on_paramsButton_clicked();

    void on_coordtransButton_clicked();

    void on_transfercoordButton_clicked();


private:
    Ui::mainWidget *ui;
    RSHD rshd_;
    bool logined_;
};

#endif // MAINWIDGET_H

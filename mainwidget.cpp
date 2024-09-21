#include "mainwidget.h"
#include "ui_mainwidget.h"
#include "qcolor.h"
#include <math.h>
#include <cmath>        // std::abs

HANDLE mainWidget::m_AlazarBoardHandle;


int mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY  = 800;
int mainWidget::MAX_NUM_SAVED_VOLUMEM_IN_MEMORY3D = 3200;
int mainWidget::m_sampleLength = 1152;
//int mainWidget::m_sampleLength = 1152;
int mainWidget::captureflag = 0;//0表示该状态不进行采集
int mainWidget::BUFFER_COUNT = 4;
int mainWidget::scanMode = 1;
int mainWidget::xMode = 0; //默认x为fast
int mainWidget::yMode = 2;
bool mainWidget::Bflag = true;
bool mainWidget::Wflag = true;//初始为不加窗
bool mainWidget::segmoveflag = true;
bool mainWidget::coarsemoveflag = true;
float mainWidget::sumedge = 0;
// 机械臂相关参量
RSHD mainWidget::pri_rshd_send;
//RSHD mainWidget::pri_rshd_recive;
#define M_PI         3.14159265358979323846
ToolInEndDesc mainWidget::tool;
int mainWidget::tooltype = 0;//默认选择法兰盘
double mainWidget::x;
double mainWidget::y;
double mainWidget::z;
double mainWidget::rx;
double mainWidget::ry;
double mainWidget::rz;
wayPoint_S waypoints;

using namespace cv;

ERROR_CODE mainWidget::ret;
ERROR_CODE mainWidget::cstatus;
cs::ICameraPtr mainWidget::camera;

float mainWidget::exposure = 3000;//设置初始曝光时间
float mainWidget::gain = 1;//设置初始增益
int mainWidget::rgbFlag = 0;//rgb默认勾选状态为0，不打开
int mainWidget::depthFlag = 0;//depth默认勾选状态为0，不打开
int mainWidget::rgbX = 100;
int mainWidget::rgbY = 100;
int mainWidget::depthX;
int mainWidget::depthY;
int mainWidget::minRange = 50;
int mainWidget::maxRange = 1000;
int mainWidget::identifyFlag = 0;//设置0为初始不识别标志
int mainWidget::labelcolor = 0;//红色=1，蓝色=2，绿色=3
float mainWidget::z1 = 0;
float mainWidget::z2 = 0;
float mainWidget::z3 = 0;
float mainWidget::z4 = 0;
float mainWidget::I3 = 0;
float mainWidget::I4 = 0;
float mainWidget::zposition = 0;
int mainWidget::moveloopidx = 1;

mainWidget::mainWidget(QWidget *parent):
    QWidget(parent),
    ui(new Ui::mainWidget)
{
    ui->setupUi(this);
//    m_cosphy = NULL;

    // 设置一下窗口显示的最小尺寸
    this->setMinimumSize(1700,800);
    // 设置textEdit文本框输出的字体的相关信息
    ui->textEdit->setFont(QFont(tr("Consolas"), 10));
    ui->SNR->setGeometry(rect().x()+450, rect().y()+20, 60, 60);
    QFont ft;
    ft.setPointSize(20);
    //ui->SNR->setFont(ft);
    ui->spinBox->setRange(0, mainWidget::m_sampleLength/2);
    ui->spinBox_2->setRange(0, mainWidget::m_sampleLength/2);
    ui->spinBox_3->setRange(40, 130);
    ui->spinBox_3->setValue(60);
    ui->spinBox_4->setRange(40, 130);
    ui->spinBox_4->setValue(100);
    ui->spinBox_5->setRange(2, 20);
    ui->spinBox_5->setValue(3);
    ui->spinBox_6->setRange(2, 20);
    ui->spinBox_6->setValue(3);


    ui->startButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/start.png);}");
    ui->startButton->setFixedSize(QSize(66, 70));
    ui->stopButton->setEnabled(FALSE);
    ui->stopButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/stopFinish.png);}");
    ui->stopButton->setFixedSize(QSize(63,70));
    ui->saveButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/iconSave.png);}");
    ui->saveButton->setFixedSize(QSize(63,70));
    ui->resetaxis->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/fit.png);}");
    ui->resetaxis->setFixedSize(QSize(63,70));
    ui->background->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/backgroung.png);}");
    ui->background->setFixedSize(QSize(63,70));
    ui->connectBotton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/connect.png);}");
    ui->connectBotton->setFixedSize(QSize(63,70));

//    ui->rangeButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/connect.png);}");
//    ui->rangeButton->setFixedSize(QSize(63,70));

//    ui->coordtransButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/connect.png);}");
//    ui->coordtransButton->setFixedSize(QSize(63,70));

//    ui->transfercoordButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/connect.png);}");
//    ui->transfercoordButton->setFixedSize(QSize(63,70));

    ui->addWindow->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/addWindow.png);}");
    ui->addWindow->setFixedSize(QSize(63,70));

    ui->textEdit->setReadOnly(true);
    ui->textEdit->setStyleSheet("QTextEdit{background-color:rgba(22,28,35,1); border:0px;color: rgb(255,255,255)}");

    ui->cosplot->setBackground(QColor(0,0,0));
    ui->cosplot->xAxis->setBasePen(QPen(Qt::white));
    ui->cosplot->xAxis->setTickPen(QPen(Qt::white));
    ui->cosplot->xAxis->setSubTickPen(QPen(Qt::white));
    ui->cosplot->yAxis->setBasePen(QPen(Qt::white));
    ui->cosplot->yAxis->setTickPen(QPen(Qt::white));
    ui->cosplot->yAxis->setSubTickPen(QPen(Qt::white));
    ui->cosplot->xAxis->setTickLabelColor(Qt::white);
    ui->cosplot->yAxis->setTickLabelColor(Qt::white);

    ui->fftplot->setBackground(QColor(0,0,0));
    ui->fftplot->xAxis->setBasePen(QPen(Qt::white));
    ui->fftplot->xAxis->setTickPen(QPen(Qt::white));
    ui->fftplot->xAxis->setSubTickPen(QPen(Qt::white));
    ui->fftplot->yAxis->setBasePen(QPen(Qt::white));
    ui->fftplot->yAxis->setTickPen(QPen(Qt::white));
    ui->fftplot->yAxis->setSubTickPen(QPen(Qt::white));
    ui->fftplot->xAxis->setTickLabelColor(Qt::white);
    ui->fftplot->yAxis->setTickLabelColor(Qt::white);

    //widget 背景色属性
    ui->tabWidget->setAttribute(Qt::WA_StyledBackground);

    //标题栏红色背景，tab选中蓝色，未选中灰色
    ui->tabWidget->setStyleSheet("QTabWidget#tabWidget{background-color:rgba(22,28,35,1);border: 0;}\
                                     QTabBar::tab{background-color:rgba(22,28,35,1);color:rgb(219,219,219); font:11pt; width: 100px;height:30px;}\
                                     QTabBar::tab::selected{background-color:rgb(32,40,50);color:rgb(219,219,219);font: bold; font-size:20px;}\
                                     QTabWidget::tab-bar{background-color:rgb(0,0,0);border-width:0px;}");
   //绿色背景
   ui->tab->setStyleSheet("QWidget#tab{"
                           "background-color:rgb(32,40,50);}");
   ui->tab_2->setStyleSheet("QWidget#tab_2{"
                            "background-color:rgb(32,40,50);}");
   ui->tab_3->setStyleSheet("QWidget#tab_3{"
                             "background-color:rgb(32,40,50);}");
   ui->tabWidget->setDocumentMode(true);

   //combobox样式
   /* 未下拉时，QComboBox的样式 */
   ui->comboBox->setView(new QListView());
   ui->comboBox->setStyleSheet("QComboBox {border: 1px solid gray;\
                                   border-radius:4px;\
                                   padding: 0px 0px 0px 10px;\
                                   color: rgba(55,55,55,1);\
                                   font: normal normal 13px;\
                                   background: transparent;\
                                   text-align: AlignHCenter;\
                                   color:rgb(240,240,240);\
                                   background-color: transparent;\
                                   border-image: url(:/new/prefix1/84.png);\
                                   height:25px;}\
                                QComboBox QAbstractItemView {outline: 0px solid gray;\
                                    border-radius:4px;\
                                    padding-top:10px;\
                                    padding-bottom:10px;\
                                    color:rgb(123,123,123);\
                                    border-image: url(:/new/prefix1/84.png);}\
                                QComboBox QAbstractItemView::item {min-height: 15px;}\
                                QComboBox QAbstractItemView::item:hover {color:rgb(255,255,255);}\
                                QComboBox QAbstractItemView::item:selected {color:rgb(255,255,255);\
                                border-image: url(:/new/prefix1/60.png);}");
        ui->comboBox_2->setView(new QListView());
        ui->comboBox_2->setStyleSheet("QComboBox {border: 1px solid gray;\
                                        border-radius:4px;\
                                        padding: 0px 0px 0px 10px;\
                                        color: rgba(55,55,55,1);\
                                        font: normal normal 13px;\
                                        background: transparent;\
                                        text-align: AlignHCenter;\
                                        color:rgb(240,240,240);\
                                        background-color: transparent;\
                                        border-image: url(:/new/prefix1/84.png);\
                                        height:25px;}\
                                     QComboBox QAbstractItemView {outline: 0px solid gray;\
                                         border-radius:4px;\
                                         padding-top:10px;\
                                         padding-bottom:10px;\
                                         color:rgb(123,123,123);\
                                         border-image: url(:/new/prefix1/84.png);}\
                                     QComboBox QAbstractItemView::item {min-height: 15px;}\
                                     QComboBox QAbstractItemView::item:hover {color:rgb(255,255,255);}\
                                     QComboBox QAbstractItemView::item:selected {color:rgb(255,255,255);\
                                     border-image: url(:/new/prefix1/60.png);}");

         ui->comboBox_3->setView(new QListView());
         ui->comboBox_3->setStyleSheet("QComboBox {border: 1px solid gray;\
                                         border-radius:4px;\
                                         padding: 0px 0px 0px 10px;\
                                         color: rgba(55,55,55,1);\
                                         font: normal normal 13px;\
                                         background: transparent;\
                                         text-align: AlignHCenter;\
                                         color:rgb(240,240,240);\
                                         background-color: transparent;\
                                         border-image: url(:/new/prefix1/84.png);\
                                         height:25px;}\
                                      QComboBox QAbstractItemView {outline: 0px solid gray;\
                                          border-radius:4px;\
                                          padding-top:10px;\
                                          padding-bottom:10px;\
                                          color:rgb(123,123,123);\
                                          border-image: url(:/new/prefix1/84.png);}\
                                      QComboBox QAbstractItemView::item {min-height: 15px;}\
                                      QComboBox QAbstractItemView::item:hover {color:rgb(255,255,255);}\
                                      QComboBox QAbstractItemView::item:selected {color:rgb(255,255,255);\
                                      border-image: url(:/new/prefix1/60.png);}");
          ui->toolbox->setView(new QListView());
          ui->toolbox->setStyleSheet("QComboBox {border: 1px solid gray;\
                                          border-radius:4px;\
                                          padding: 0px 0px 0px 10px;\
                                          color: rgba(55,55,55,1);\
                                          font: normal normal 13px;\
                                          background: transparent;\
                                          text-align: AlignHCenter;\
                                          color:rgb(240,240,240);\
                                          background-color: transparent;\
                                          border-image: url(:/new/prefix1/84.png);\
                                          height:25px;}\
                                       QComboBox QAbstractItemView {outline: 0px solid gray;\
                                           border-radius:4px;\
                                           padding-top:10px;\
                                           padding-bottom:10px;\
                                           color:rgb(123,123,123);\
                                           border-image: url(:/new/prefix1/84.png);}\
                                       QComboBox QAbstractItemView::item {min-height: 15px;}\
                                       QComboBox QAbstractItemView::item:hover {color:rgb(255,255,255);}\
                                       QComboBox QAbstractItemView::item:selected {color:rgb(255,255,255);\
                                       border-image: url(:/new/prefix1/60.png);}");
       ui->colorcombo->setView(new QListView());
       ui->colorcombo->setStyleSheet("QComboBox {border: 1px solid gray;\
                                       border-radius:4px;\
                                       padding: 0px 0px 0px 10px;\
                                       color: rgba(55,55,55,1);\
                                       font: normal normal 13px;\
                                       background: transparent;\
                                       text-align: AlignHCenter;\
                                       color:rgb(240,240,240);\
                                       background-color: transparent;\
                                       border-image: url(:/new/prefix1/84.png);\
                                       height:25px;}\
                                    QComboBox QAbstractItemView {outline: 0px solid gray;\
                                        border-radius:4px;\
                                        padding-top:10px;\
                                        padding-bottom:10px;\
                                        color:rgb(123,123,123);\
                                        border-image: url(:/new/prefix1/84.png);}\
                                    QComboBox QAbstractItemView::item {min-height: 15px;}\
                                    QComboBox QAbstractItemView::item:hover {color:rgb(255,255,255);}\
                                    QComboBox QAbstractItemView::item:selected {color:rgb(255,255,255);\
                                    border-image: url(:/new/prefix1/60.png);}");
    ui->label->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_2->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_3->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_4->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_5->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_6->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_7->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_8->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_9->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_10->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_11->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_12->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_13->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_14->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_15->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_16->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_17->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_18->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_19->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_20->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_21->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_22->setStyleSheet("QLabel{background-color:rgb(32,40,50);background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_23->setStyleSheet("QLabel{background-color:rgb(32,40,50);background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_24->setStyleSheet("QLabel{background-color:rgb(32,40,50);background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_25->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_26->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_27->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_28->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_29->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_30->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_31->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_x->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 12px;}");
    ui->label_y->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 12px;}");
    ui->label_z->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 12px;}");
    ui->label_rx->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 12px;}");
    ui->label_ry->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 12px;}");
    ui->label_rz->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 12px;}");
    ui->label_32->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_33->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_34->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_35->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_36->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_37->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_38->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_39->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_40->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_41->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_42->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_43->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_44->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_45->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_46->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_47->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_48->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_49->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_50->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_51->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->label_52->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;}");
    ui->line_5->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_2->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_3->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_4->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_5->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_6->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_7->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_8->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_9->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_10->setStyleSheet("Line{color: rgb(0,0,0);}");
    ui->line_11->setStyleSheet("Line{color: rgb(0,0,0);}");

    ui->SNR->setStyleSheet("QLabel{background-color:rgb(32,40,50);font-weight: bold;color: rgb(255,255,255);font: bold 14px;background-color:rgba(55,55,55,0);}");
    ui->spinBox->setStyleSheet("QSpinBox {background-color:rgb(55,55,55);\
            font: 75 11pt;\
            color:rgb(240,240,240);\
            border-radius:4px; height:25px;\
            width:100;}\
        QSpinBox:hover{background-color:rgb(35,35,35);}\
        QSpinBox:up-button{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
        QSpinBox:down-button{subcontrol-origin:border;\
            subcontrol-position:left;\
            image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
        QSpinBox:up-button:hover{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right_pressed.png);}\
        QSpinBox:down-button:hover{subcontrol-position:left;\
            image: url(:/new/prefix1/left_pressed.png);}\
        QSpinBox:up-button:pressed{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right_pressed.png);}\
        QSpinBox:down-button:pressed{subcontrol-position:left;\
            image: url(:/Resource/spinbox/left_pressed.png);}");
    ui->spinBox_2->setStyleSheet("QSpinBox {background-color:rgb(55,55,55);\
            font: 75 11pt;\
            color:rgb(240,240,240);\
            border-radius:4px; height:25px;\
            width:100;}\
        QSpinBox:hover{background-color:rgb(35,35,35);}\
        QSpinBox:up-button{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
        QSpinBox:down-button{subcontrol-origin:border;\
            subcontrol-position:left;\
            image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
        QSpinBox:up-button:hover{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right_pressed.png);}\
        QSpinBox:down-button:hover{subcontrol-position:left;\
            image: url(:/new/prefix1/left_pressed.png);}\
        QSpinBox:up-button:pressed{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right_pressed.png);}\
        QSpinBox:down-button:pressed{subcontrol-position:left;\
            image: url(:/Resource/spinbox/left_pressed.png);}");
     ui->spinBox_3->setStyleSheet("QSpinBox {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:25;}\
         QSpinBox:hover{background-color:rgb(35,35,35);}\
         QSpinBox:up-button{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
         QSpinBox:down-button{subcontrol-origin:border;\
             subcontrol-position:left;\
             image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
         QSpinBox:up-button:hover{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QSpinBox:down-button:hover{subcontrol-position:left;\
             image: url(:/new/prefix1/left_pressed.png);}\
         QSpinBox:up-button:pressed{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QSpinBox:down-button:pressed{subcontrol-position:left;\
             image: url(:/Resource/spinbox/left_pressed.png);}");
     ui->spinBox_4->setStyleSheet("QSpinBox {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:25;}\
         QSpinBox:hover{background-color:rgb(35,35,35);}\
         QSpinBox:up-button{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
         QSpinBox:down-button{subcontrol-origin:border;\
             subcontrol-position:left;\
             image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
         QSpinBox:up-button:hover{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QSpinBox:down-button:hover{subcontrol-position:left;\
             image: url(:/new/prefix1/left_pressed.png);}\
         QSpinBox:up-button:pressed{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QSpinBox:down-button:pressed{subcontrol-position:left;\
             image: url(:/Resource/spinbox/left_pressed.png);}");
     ui->spinBox_5->setStyleSheet("QSpinBox {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:25;}\
         QSpinBox:hover{background-color:rgb(35,35,35);}\
         QSpinBox:up-button{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
         QSpinBox:down-button{subcontrol-origin:border;\
             subcontrol-position:left;\
             image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
         QSpinBox:up-button:hover{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QSpinBox:down-button:hover{subcontrol-position:left;\
             image: url(:/new/prefix1/left_pressed.png);}\
         QSpinBox:up-button:pressed{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QSpinBox:down-button:pressed{subcontrol-position:left;\
             image: url(:/Resource/spinbox/left_pressed.png);}");
     ui->spinBox_6->setStyleSheet("QSpinBox {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:25;}\
         QSpinBox:hover{background-color:rgb(35,35,35);}\
         QSpinBox:up-button{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
         QSpinBox:down-button{subcontrol-origin:border;\
             subcontrol-position:left;\
             image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
         QSpinBox:up-button:hover{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QSpinBox:down-button:hover{subcontrol-position:left;\
             image: url(:/new/prefix1/left_pressed.png);}\
         QSpinBox:up-button:pressed{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QSpinBox:down-button:pressed{subcontrol-position:left;\
             image: url(:/Resource/spinbox/left_pressed.png);}");
     ui->doubleSpinBoxa1->setStyleSheet("QDoubleSpinBox {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:25;}\
         QDoubleSpinBox:hover{background-color:rgb(35,35,35);}\
         QDoubleSpinBox:up-button{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
         QDoubleSpinBox:down-button{subcontrol-origin:border;\
             subcontrol-position:left;\
             image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
         QDoubleSpinBox:up-button:hover{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QDoubleSpinBox:down-button:hover{subcontrol-position:left;\
             image: url(:/new/prefix1/left_pressed.png);}\
         QDoubleSpinBox:up-button:pressed{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QDoubleSpinBox:down-button:pressed{subcontrol-position:left;\
             image: url(:/Resource/spinbox/left_pressed.png);}");
     ui->doubleSpinBoxa2->setStyleSheet("QDoubleSpinBox {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:25;}\
         QDoubleSpinBox:hover{background-color:rgb(35,35,35);}\
         QDoubleSpinBox:up-button{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
         QDoubleSpinBox:down-button{subcontrol-origin:border;\
             subcontrol-position:left;\
             image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
         QDoubleSpinBox:up-button:hover{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QDoubleSpinBox:down-button:hover{subcontrol-position:left;\
             image: url(:/new/prefix1/left_pressed.png);}\
         QDoubleSpinBox:up-button:pressed{subcontrol-origin:border;\
             subcontrol-position:right;\
             image: url(:/new/prefix1/right_pressed.png);}\
         QDoubleSpinBox:down-button:pressed{subcontrol-position:left;\
             image: url(:/Resource/spinbox/left_pressed.png);}");
     ui->doubleSpinBoxw0->setStyleSheet("QDoubleSpinBox {background-color:rgb(55,55,55);\
            font: 75 11pt;\
            color:rgb(240,240,240);\
            border-radius:4px; height:25px;\
            width:25;}\
        QDoubleSpinBox:hover{background-color:rgb(35,35,35);}\
        QDoubleSpinBox:up-button{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right.png);width: 20px;height: 20px;}\
        QDoubleSpinBox:down-button{subcontrol-origin:border;\
            subcontrol-position:left;\
            image: url(:/new/prefix1/left.png);width: 20px;height: 20px;}\
        QDoubleSpinBox:up-button:hover{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right_pressed.png);}\
        QDoubleSpinBox:down-button:hover{subcontrol-position:left;\
            image: url(:/new/prefix1/left_pressed.png);}\
        QDoubleSpinBox:up-button:pressed{subcontrol-origin:border;\
            subcontrol-position:right;\
            image: url(:/new/prefix1/right_pressed.png);}\
        QDoubleSpinBox:down-button:pressed{subcontrol-position:left;\
            image: url(:/Resource/spinbox/left_pressed.png);}");
     ui->spinBox->setAlignment(Qt::AlignHCenter);
     ui->spinBox_2->setAlignment(Qt::AlignHCenter);
     ui->spinBox_3->setAlignment(Qt::AlignHCenter);
     ui->spinBox_4->setAlignment(Qt::AlignHCenter);
     ui->spinBox_5->setAlignment(Qt::AlignHCenter);
     ui->spinBox_6->setAlignment(Qt::AlignHCenter);
     ui->doubleSpinBoxa1->setAlignment(Qt::AlignHCenter);
     ui->doubleSpinBoxa2->setAlignment(Qt::AlignHCenter);
     ui->amplitude->setAlignment(Qt::AlignHCenter);
     ui->frameRate->setAlignment(Qt::AlignHCenter);
     ui->repeatnum->setAlignment(Qt::AlignHCenter);
     ui->Bscanlines->setAlignment(Qt::AlignHCenter);
     ui->dutycycle->setAlignment(Qt::AlignHCenter);
     ui->doubleSpinBoxw0->setAlignment(Qt::AlignHCenter);
     ui->le_IP->setAlignment(Qt::AlignHCenter);
     ui->maxVelocity->setAlignment(Qt::AlignHCenter);
     ui->maxAcc->setAlignment(Qt::AlignHCenter);
     ui->xvalue->setAlignment(Qt::AlignHCenter);
     ui->yvalue->setAlignment(Qt::AlignHCenter);
     ui->zvalue->setAlignment(Qt::AlignHCenter);
     ui->rxvalue->setAlignment(Qt::AlignHCenter);
     ui->ryvalue->setAlignment(Qt::AlignHCenter);
     ui->rzvalue->setAlignment(Qt::AlignHCenter);

     ui->amplitude->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:100;}");
     ui->frameRate->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:100;}");
     ui->repeatnum->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:100;}");
     ui->Bscanlines->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:100;}");
     ui->dutycycle->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
             font: 75 11pt;\
             color:rgb(240,240,240);\
             border-radius:4px; height:25px;\
             width:100;}");
     ui->connectDA->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                  font: 75 11pt;\
                                  color:rgb(240,240,240);\
                                  border-radius:4px; height:25px;\
                                  width:100;\
                                  font-weight: bold}\
                                  QPushButton:hover{background-color:rgb(35,35,35);}");
      ui->le_IP->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->maxAcc->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->maxVelocity->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->xvalue->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->yvalue->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->zvalue->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->rxvalue->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->ryvalue->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->rzvalue->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->gaintext->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->expotext->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->minrange->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->maxrange->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->rgbx->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->rgby->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->depthx->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->depthy->setStyleSheet("QLineEdit {background-color:rgb(55,55,55);\
              font: 75 11pt;\
              color:rgb(240,240,240);\
              border-radius:4px; height:25px;\
              width:100;}");
      ui->rgbCheckBox->setStyleSheet("QCheckBox {\
                                   font: 75 11pt;\
                                   color:rgb(240,240,240);\
                                   background-color:rgb(32,40,50);\
                                   height:25px;\
                                   width:100;}");

       ui->depthCheckBox->setStyleSheet("QCheckBox {\
                                    font: 75 11pt;\
                                    color:rgb(240,240,240);\
                                    background-color:rgb(32,40,50);\
                                    height:25px;\
                                    width:100;}");
      ui->pb_login->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                   font: 75 11pt;\
                                   color:rgb(240,240,240);\
                                   border-radius:4px; height:25px;\
                                   width:100;\
                                   font-weight: bold}\
                                   QPushButton:hover{background-color:rgb(35,35,35);}");
       ui->logoutButton->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                    font: 75 11pt;\
                                    color:rgb(240,240,240);\
                                    border-radius:4px; height:25px;\
                                    width:100;\
                                    font-weight: bold}\
                                    QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->moveO->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                     font: 75 11pt;\
                                     color:rgb(240,240,240);\
                                     border-radius:4px; height:25px;\
                                     width:100;\
                                     font-weight: bold}\
                                     QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->saveData->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                  font: 75 11pt;\
                                  color:rgb(240,240,240);\
                                  border-radius:4px; height:25px;\
                                  width:100;\
                                  font-weight: bold}\
                                  QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->moveTo->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                font: 75 11pt;\
                                color:rgb(240,240,240);\
                                border-radius:4px; height:25px;\
                                width:100;\
                                font-weight: bold}\
                                QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->opencameraButton->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                font: 75 11pt;\
                                color:rgb(240,240,240);\
                                border-radius:4px; height:25px;\
                                width:100;\
                                font-weight: bold}\
                                QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->connectcameraButton->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                font: 75 11pt;\
                                color:rgb(240,240,240);\
                                border-radius:4px; height:25px;\
                                width:100;\
                                font-weight: bold}\
                                QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->identifylabel->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                font: 75 11pt;\
                                color:rgb(240,240,240);\
                                border-radius:4px; height:25px;\
                                width:100;\
                                font-weight: bold}\
                                QPushButton:hover{background-color:rgb(35,35,35);}");

        ui->paramsButton->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                     font: 75 11pt;\
                                     color:rgb(240,240,240);\
                                     border-radius:4px; height:25px;\
                                     width:100;\
                                     font-weight: bold}\
                                     QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->coordtransButton->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                              font: 75 11pt;\
                              color:rgb(240,240,240);\
                              border-radius:4px; height:25px;\
                              width:100;\
                              font-weight: bold}\
                              QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->transfercoordButton->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                   font: 75 11pt;\
                                   color:rgb(240,240,240);\
                                   border-radius:4px; height:25px;\
                                   width:100;\
                                   font-weight: bold}\
                                   QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->closecameraButton->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                  font: 75 11pt;\
                                  color:rgb(240,240,240);\
                                  border-radius:4px; height:25px;\
                                  width:100;\
                                  font-weight: bold}\
                                  QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->closeidentify->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                 font: 75 11pt;\
                                 color:rgb(240,240,240);\
                                 border-radius:4px; height:25px;\
                                 width:100;\
                                 font-weight: bold}\
                                 QPushButton:hover{background-color:rgb(35,35,35);}");

// 初始化采集卡
        U32 boardCount = AlazarBoardsInSystemBySystemID(1);

        if (boardCount == 1)
        {
            ui->textEdit->append("系统中找到采集卡 ");
        }

        if (boardCount == 0)
        {
            ui->textEdit->append("系统中没有找到采集卡 ");
        }

        mainWidget::m_AlazarBoardHandle = AlazarGetBoardBySystemID(1, 1);
        if (!mainWidget::m_AlazarBoardHandle)
        {
            ui->textEdit->append("系统中没有找到采集卡 ");
        }

        // 绘图库的初始化：
        ui->cosplot->addGraph();
        //设置坐标轴标签名称
        //ui->cosplot->xAxis->setLabel("x");
        //ui->cosplot->yAxis->setLabel("y");

        //设置坐标轴显示范围,否则我们只能看到默认的范围
        ui->cosplot->xAxis->setRange(0,mainWidget::m_sampleLength);
        ui->cosplot->yAxis->setRange(-2048,2048);
        ui->cosplot->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);


        ui->fftplot->addGraph();

        //设置坐标轴标签名称
        //ui->fftplot->xAxis->setLabel("x");
        //ui->fftplot->yAxis->setLabel("f");

        //设置坐标轴显示范围,否则我们只能看到默认的范围
        ui->fftplot->xAxis->setRange(1,mainWidget::m_sampleLength/2);
        ui->fftplot->yAxis->setRange(20,120);
        ui->fftplot->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);

        PrintBoardInfo();

        //添加数据
        //连接DA卡和初始化相关的系数
        pDAUSB3020 = new DA_USB3020(200319L, 10, 100, 0, 2);
        if (pDAUSB3020->isConnected())
        {
            ui->textEdit->append("Connect Success");
            Voltage = 1150; //mV 修改振镜电压
            Frequency = 225.4; // Hz
            duty_cycle = 0.9;
            BScanlines = 800; // Frames
            //pDAUSB3020->CalculateLength(Voltage, Frequency, duty_cycle, BScanlines);
            ui->textEdit->append("DA卡初始化成功");
        }
        else
        {
            ui->textEdit->append("DA卡初始化失败");
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

        // 实例OCT线程，先不启动该线程
        firstThread = new QThread;
        ssoctThread = new mythread;
        ssoctThread->moveToThread(firstThread);
        connect(ui->startButton,SIGNAL(clicked()),ssoctThread, SLOT(entry()));
        // 实例机械臂线程，先不启动该线程
        secondThread = new QThread;
        rbThread = new robotthread;
        ssoctThread->moveToThread(secondThread);
        connect(ui->moveO,SIGNAL(clicked()),rbThread, SLOT(example_moveJ()));
        connect(ui->moveTo,SIGNAL(clicked()),rbThread, SLOT(movetoP()));
        connect(ui->fmove,SIGNAL(clicked()),rbThread, SLOT(fmoveto()));
        connect(ui->coarsefButton,SIGNAL(clicked()),rbThread, SLOT(coarsemoveto()));

        //connect(this, &mainWidget::sendrobotarm, rbThread, SLOT(SLOT(movetoP())));
        connect(rbThread, SIGNAL(IKfail(int)), this, SLOT(recieveIK(int)));
        // 实例相机线程，先不启动该线程
        qRegisterMetaType<cv::Mat>("cv::Mat");
        thirdThread = new QThread;
        MyCamThread = new camerathread;
        MyCamThread->moveToThread(thirdThread);
        connect(&fps_timer, SIGNAL(timeout()), MyCamThread, SLOT(startgetframe()));
        connect(MyCamThread,SIGNAL(updateImage(cv::Mat)),this,SLOT(receiveimage(cv::Mat)));
        connect(MyCamThread,SIGNAL(updateImage2(cv::Mat)),this,SLOT(receiveimage2(cv::Mat)), Qt::DirectConnection);
        connect(MyCamThread,SIGNAL(updateImage3(cv::Mat, cv::Mat)),this,SLOT(receiveimage3(cv::Mat, cv::Mat)), Qt::DirectConnection);
        fps_timer.setInterval(800);//ms

        //初始化机械臂相关参数
        logined_ = true;

        timer = new QTimer(this);
        connect(timer,SIGNAL(timeout()),this,SLOT(displayRobotData()));
        //connect(ui->moveO,SIGNAL(clicked()),MyrbThread, SLOT(example_moveJ()));

        CoordCalibrateByJointAngleAndTool auboRobotCoordInfo;
        auboRobotCoordInfo.toolDesc.toolInEndOrientation.w = 1;
        auboRobotCoordInfo.toolDesc.toolInEndOrientation.x = 0;
        auboRobotCoordInfo.toolDesc.toolInEndOrientation.y = 0;
        auboRobotCoordInfo.toolDesc.toolInEndOrientation.z = 0;
        auboRobotCoordInfo.toolDesc.toolInEndPosition.x = 0;
        auboRobotCoordInfo.toolDesc.toolInEndPosition.y = 0;
        auboRobotCoordInfo.toolDesc.toolInEndPosition.z = 0;
        auboRobotCoordInfo.coordType = BaseCoordinate;


}

void mainWidget::on_connectDA_clicked()
{
    Voltage = ui->amplitude->text().toDouble();//添加数据
    Frequency = ui->frameRate->text().toDouble();//添加数据
    duty_cycle = ui->dutycycle->text().toDouble();//添加数据
    BScanlines = ui->Bscanlines->text().toDouble();//添加数据
    if (Voltage < 400 && Voltage > 2000)
    {
        QMessageBox *msgBox = new QMessageBox();
        msgBox->setStyleSheet("background-color:black");
        QMessageBox::information(this, "information","<font size='18' color='white'>The amplitude value can't exceed range 400-2000.</font>");

        Voltage = 1150;
    }
    if (Frequency > 400)
    {
        QMessageBox *msgBox = new QMessageBox();
        msgBox->setStyleSheet("background-color:black");
        QMessageBox::information(this, "information","<font size='18' color='white'>The frequency value can't exceed 400.</font>");

        Frequency = 225.40;
    }


    pDAUSB3020->DisableDA();
    pDAUSB3020->ReleaseDA();
    pDAUSB3020->CalculateDAdata(Voltage, Frequency, duty_cycle, BScanlines,mainWidget::xMode, mainWidget::yMode);
    ui->textEdit->append("start writing");

    bool br = pDAUSB3020->WriteDataToDA();
    if (br)
    {
        ui->textEdit->append("DA卡连接成功！ ");
    }
    else
    {
        ui->textEdit->append("DA卡连接失败！ ");
    }
    return;
}

void mainWidget::PrintBoardInfo()
{
    U8 sdkMajor, sdkMinor, sdkRevision;
    RETURN_CODE retCode = AlazarGetSDKVersion(&sdkMajor, &sdkMinor, &sdkRevision);
    ui->textEdit->append("SDK version  = " + QVariant(sdkMajor).toString() + "." + QVariant(sdkMinor).toString() + "." + QVariant(sdkRevision).toString());
    U32 boardType = AlazarGetBoardKind(mainWidget::m_AlazarBoardHandle);
    if (boardType == ATS_NONE || boardType >= ATS_LAST)
    {

        ui->textEdit->append("未知的alazar卡的种类" + QString::number(boardType));
    }
    U8 driverMajor, driverMinor, driverRev;
    AlazarGetDriverVersion(&driverMajor, &driverMinor, &driverRev);
    ui->textEdit->append("Board type =" + QString::number(boardType));
    //wxLogMessage("Board count = %u", boardCount);
    ui->textEdit->append("Driver version ="  + QVariant(driverMajor).toString() + "." + QVariant(driverMinor).toString() + "." + QVariant(driverRev).toString());
    U32 samplesPerChannel;
    BYTE bitsPerSample;
    retCode = AlazarGetChannelInfo(mainWidget::m_AlazarBoardHandle, &samplesPerChannel, &bitsPerSample);
    U32 aspocType;
    retCode = AlazarQueryCapability(mainWidget::m_AlazarBoardHandle, ASOPC_TYPE, 0, &aspocType);
    BYTE cpldMajor;
    BYTE cpldMinor;
    retCode = AlazarGetCPLDVersion(mainWidget::m_AlazarBoardHandle, &cpldMajor, &cpldMinor);
    U32 serialNumber;
    retCode = AlazarQueryCapability(mainWidget::m_AlazarBoardHandle, GET_SERIAL_NUMBER, 0, &serialNumber);
    U32 latestCalDate;
    retCode = AlazarQueryCapability(mainWidget::m_AlazarBoardHandle, GET_LATEST_CAL_DATE, 0, &latestCalDate);

    ui->textEdit->append("Serial number =" + QString::number(serialNumber));
    ui->textEdit->append("Bits per sample = " + QString::number(bitsPerSample));
    ui->textEdit->append("Max samples per channel = " + QString::number(samplesPerChannel));
    ui->textEdit->append("Latest calibration date = " + QString::number(latestCalDate));

    U32 linkSpeed;
    retCode = AlazarQueryCapability(mainWidget::m_AlazarBoardHandle, GET_PCIE_LINK_SPEED, 0, &linkSpeed);

    U32 linkWidth;
    retCode = AlazarQueryCapability(m_AlazarBoardHandle, GET_PCIE_LINK_WIDTH, 0, &linkWidth);

    ui->textEdit->append("PCIe link speed =" + QString::number(2.5 * linkSpeed) + "Gbps");
    ui->textEdit->append("PCIe link width =" + QString::number(linkWidth) + "lanes");

    float fpgaTemperature_degreesC;
    retCode = AlazarGetParameterUL(mainWidget::m_AlazarBoardHandle, CHANNEL_ALL,
        GET_FPGA_TEMPERATURE, (U32*) &fpgaTemperature_degreesC);
    ui->textEdit->append("FPGA temperature =" + QString::number(fpgaTemperature_degreesC) + "degree");
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
    secondThread->quit();
    secondThread->wait();
//    thirdThread->quit();
//    thirdThread->wait();
//    mainWidget::camera->disconnect();
    killTimer(timerId1);
    killTimer(timerId2);
    killTimer(timerId3);


    delete ui;
}

//下拉框，区别扫描模式
void mainWidget::on_comboBox_activated(const QString &arg1)
{
    if(arg1 == "1D scan")
    {
        mainWidget::scanMode = 1;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
    if(arg1 == "2D cross scan")
    {
        mainWidget::scanMode = 10;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
    if(arg1 == "2D repeat")
    {
        mainWidget::scanMode = 2;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
    if(arg1 == "2D angio")
    {
        mainWidget::scanMode = 22;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
    if(arg1 == "3D scan")
    {
        mainWidget::scanMode = 3;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
    if(arg1 == "3D angio")
    {
        mainWidget::scanMode = 32;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
    if(arg1 == "2D fchange")
    {
        mainWidget::scanMode = 23;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
    if(arg1 == "2D coarsefc")
    {
        mainWidget::scanMode = 24;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
}

void mainWidget::on_comboBox_2_activated(const QString &arg1)
{
    if(arg1 == "X fast")
    {
        mainWidget::xMode = 0;
        mainWidget::yMode = 2;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
    if(arg1 == "Y fast")
    {
        mainWidget::xMode = 2;
        mainWidget::yMode = 0;
        ui->textEdit->append("The current mode is: " + QVariant(arg1).toString());
    }
}

void mainWidget::displayRobotData()
{
    toolBox();
    rs_get_current_waypoint(mainWidget::pri_rshd_send, &waypoints);
    if(mainWidget::tooltype == 0)
    {
    ui->label_x->setNum(waypoints.cartPos.position.x);
    ui->label_y->setNum(waypoints.cartPos.position.y);
    ui->label_z->setNum(waypoints.cartPos.position.z);
    Rpy Eularangle;
    rs_quaternion_to_rpy(mainWidget::pri_rshd_send, &waypoints.orientation ,&Eularangle);
    ui->label_rx->setNum(Eularangle.rx * 180 / M_PI);
    ui->label_ry->setNum(Eularangle.ry * 180 / M_PI);
    ui->label_rz->setNum(Eularangle.rz * 180 / M_PI);
    }
    if(mainWidget::tooltype == 1)
    {
        Pos pos_onuser;
        Ori ori_onuser;

        // 基座标系转用户坐标系
        current_Coord.coordType = BaseCoordinate;
        rs_base_to_user(mainWidget::pri_rshd_send, &waypoints.cartPos.position, &waypoints.orientation, &current_Coord, &mainWidget::tool, &pos_onuser, &ori_onuser);


        Rpy rpy;
        rs_quaternion_to_rpy(mainWidget::pri_rshd_send, &ori_onuser, &rpy);

        ui->label_rx->setNum(rpy.rx*180.0/M_PI);
        ui->label_ry->setNum(rpy.ry*180.0/M_PI);
        ui->label_rz->setNum(rpy.rz*180.0/M_PI);
        ui->label_x->setNum(pos_onuser.x);
        ui->label_y->setNum(pos_onuser.y);
        ui->label_z->setNum(pos_onuser.z);
    }

}



void mainWidget::PrepareProcessing1D()
{
    mkl_free_buffers();
    mkl_thread_free_buffers();

}


void mainWidget::on_startButton_clicked()
{
    ui->startButton->setEnabled(FALSE);
    ui->startButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/start2.png);}");
    ui->stopButton->setEnabled(TRUE);
    ui->stopButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/stopInitial.png);}");
    mainWidget::captureflag = 1;   
    OCTplots();
    mainWidget::z1 = 0;
    mainWidget::z2 = 0;
    mainWidget::z3 = 0;
    mainWidget::z4 = 0;
    mainWidget::I3 = 0;
    mainWidget::I4 = 0;
    mainWidget::moveloopidx = 1;

}

void mainWidget::on_connectBotton_clicked()
{
    firstThread->start();
    if (!ssoctThread->StartAlazarADcapture())
    {
        ui->textEdit->append("设置启动采集卡失败！");
        return;
    }

    ui->textEdit->append("连接采集卡成功！");
    ui->connectBotton->setEnabled(FALSE);
    ui->connectBotton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/disconnect.png);}");
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


        if(mainWidget::scanMode == 23)// | mainWidget::scanMode == 32)
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


            memcpy(m_curDisplayData, ssoctThread->m_volumeMemBuffer[availbleIndex], mainWidget::m_sampleLength * 800 * sizeof(short));

            p_F2 = new float[mainWidget::m_sampleLength * 800];
            for (int j = 0; j < 800; j++)
            {
                for (int i = 0; i < mainWidget::m_sampleLength; i++)
                {
                    p_F2[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayData[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
                }
            }

            fftBfcous(p_F2);

            delete [] m_curDisplayData;

            delete [] p_F2;
        }

        if(mainWidget::scanMode == 24)// | mainWidget::scanMode == 32)
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


            memcpy(m_curDisplayData, ssoctThread->m_volumeMemBuffer[availbleIndex], mainWidget::m_sampleLength * 800 * sizeof(short));

            p_F2 = new float[mainWidget::m_sampleLength * 800];
            for (int j = 0; j < 800; j++)
            {
                for (int i = 0; i < mainWidget::m_sampleLength; i++)
                {
                    p_F2[j * mainWidget::m_sampleLength + i ] = ((m_curDisplayData[j * mainWidget::m_sampleLength + i ] >> 4) - m_BG[i] - 2048) * hammingwindow[i];
                }
            }

            fftBfcous2(p_F2);

            delete [] m_curDisplayData;

            delete [] p_F2;
        }


    }


}

bool mainWidget::Robot_Login()
{
    if(rs_initialize() != RS_SUCC) {
        qDebug() << "初始化失败 ";
        return false;
    }
    if(rs_create_context(&mainWidget::pri_rshd_send) != RS_SUCC) {
        qDebug() << "创建上下文失败 ";
        return false;
    }
    if(rs_login(mainWidget::pri_rshd_send, ui->le_IP->text().toStdString().c_str(), 8899) != RS_SUCC) {
        qDebug() << "登录失败 ";
        return false;
    }
    qDebug() << "登录成功 ";
    //如果有真实机，才startup
    aubo_robot_namespace::RobotWorkMode mode;
    if(RS_SUCC != rs_get_work_mode(mainWidget::pri_rshd_send, &mode)) {
        qDebug() << "get robot work mode failed";
        return false;
    }
    if(mode == aubo_robot_namespace::RobotModeReal) {
//    if(1) {
        qDebug() << "real robot ";
        bool result = false;
        //工具的动力学参数和运动学参数
        ToolDynamicsParam tool_dynamics = {0};
        //机械臂碰撞等级
        uint8 colli_class = 6;
        //机械臂启动是否读取姿态（默认开启）
        bool read_pos = true;
        //机械臂静态碰撞检测（默认开启）
        bool static_colli_detect = true;
        //机械臂最大加速度（系统自动控制，默认为30000)
        int board_maxacc = 30000;
        //机械臂服务启动状态
        ROBOT_SERVICE_STATE state = ROBOT_SERVICE_READY;

        if (rs_robot_startup(mainWidget::pri_rshd_send, &tool_dynamics, colli_class, read_pos, static_colli_detect, board_maxacc, &state)
            == RS_SUCC)
        {
            result = true;
            qDebug() << "robottest";
            //std::cout<<"call robot startup succ, robot state:"<<state<<std::endl;
        }
        else
        {
            //std::cerr<<"robot startup failed"<<std::endl;
        }
    }
    else {
        qDebug() << "simulate robot";
    }


    timer->start(100);
    secondThread->start();
    return true;

}

bool mainWidget::logout(RSHD rshd)
{
    return rs_logout(rshd)==RS_SUCC ? true : false;
}

void mainWidget::toolBox()
{
    if(ui->toolbox->currentText() == "flange")
    {
        mainWidget::tooltype = 0;
        //qDebug() << "flange";
    }
    if(ui->toolbox->currentText() == "needle")
    {
        mainWidget::tooltype = 1;
        //qDebug() << "needle";
        //ToolInEndDesc tool;
        mainWidget::tool.toolInEndPosition.x = -0.000275;
        mainWidget::tool.toolInEndPosition.y = 0.000330;
        mainWidget::tool.toolInEndPosition.z = 0.141153;
        //转为四元数
        Ori toolorient;
        // 初始其eular 角度
        Rpy toolEularangle;
        toolEularangle.rx = 0.310027 / 180 * M_PI;
        toolEularangle.ry = 0.269978/ 180 * M_PI;
        toolEularangle.rz = 0.099981/ 180 * M_PI;
        rs_rpy_to_quaternion(mainWidget::pri_rshd_send, &toolEularangle, &toolorient);
        mainWidget::tool.toolInEndOrientation.w = toolorient.w;
        mainWidget::tool.toolInEndOrientation.x = toolorient.x;
        mainWidget::tool.toolInEndOrientation.y = toolorient.y;
        mainWidget::tool.toolInEndOrientation.z = toolorient.z;

        rs_set_tool_end_param(mainWidget::pri_rshd_send, &mainWidget::tool);
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

void mainWidget::fftBfcous(float *numbers)
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
        plotBfocus(numberslog);

        mkl_free(numberslog);
        mkl_free(m_cosphy);
        mkl_free(m_sinphy);

//        killTimer(timerId3);
//        firstThread->quit();
//        firstThread->wait();
//        pDAUSB3020->DisableDA();
//        pDAUSB3020->ReleaseDA();
//        pDAUSB3020->StopScan();
}

void mainWidget::plotBfocus(float *num)
{
    // 考虑得到图像的上边界
    // 首先将num数组转为cv::Mat格式的数据

    // 然后我们对这个数据img进行高斯滤波


    // 考虑log图像的显示；
    double tmp1;
    double tmp2;
    tmp1 = ui->spinBox_3->text().toDouble();   //添加数据
    tmp2 = ui->spinBox_4->text().toDouble();   //添加数据
    if (tmp2 > tmp1)
    {

        cv::Mat img(mainWidget::m_sampleLength/2, 800, CV_32F);

            for (int j = 0; j < 800; j++)
            {
                for (int i = 0; i < mainWidget::m_sampleLength/2; i++)
                {

                        img.at<float>(i, j) = num[j * mainWidget::m_sampleLength + i];
                }
            }
            cv::Mat img2(mainWidget::m_sampleLength/2, 720, CV_32F);
            img2 = img(Range(1, mainWidget::m_sampleLength/2), Range(1, 720));

            cv::Mat gsout;

            cv::GaussianBlur(img2, gsout, cv::Size(17,17), 0, 0);

            cv::Mat line1;
            cv::Mat edge = cv::Mat::zeros(mainWidget::m_sampleLength/2, 720,CV_32F);
            int edge2[720] = {0};
            double maxvalue = 0, minvalue = 0;
            Point max_ind, min_ind;
            double thrsh_ratio = 0.9;
            for (int j = 1; j < 719; j++)
            {
                maxvalue = 0, minvalue = 0;
                line1 = gsout.colRange(j, j+1).clone();
                minMaxLoc(line1, &minvalue, &maxvalue,&min_ind,&max_ind);


                for (int i = 5; i < 575; i++)
                {

                        if(gsout.at<float>(i, j - 1) > maxvalue * thrsh_ratio)
                        {
                            edge.at<float>(i, j) = 255;
                            edge2[j - 1] = i;
                            break;
                        }
                       //break;
                }
            }
            qDebug()<<"edge detect";
            if(mainWidget::segmoveflag == true)
            {

                int indexsum = 0;
                for(int j = 0; j < 720; j++)
                {
                    if(edge2[j] > 6)
                    {
                        indexsum = indexsum +1;
                        mainWidget::sumedge = mainWidget::sumedge + edge2[j] - 150;
                    }
                }
                mainWidget::sumedge = -(mainWidget::sumedge/(indexsum + 1))* 5 / mainWidget::m_sampleLength/2 / 1000;
                qDebug()<<"sumedge"<<mainWidget::sumedge;
            }
            //mainWidget::segmoveflag = false;
//            // 考虑血流成像的C++程序的写作；
//            // 先不封装成函数
            Mat Temp;

            edge.convertTo(Temp, CV_8UC1);


            QImage Qtemp = putImage(Temp);
            ui->label2D->setPixmap(QPixmap::fromImage(Qtemp.scaled(ui->label2D->size())));
    }
    else
    {
        return;
    }
}

void mainWidget::fftBfcous2(float *numbers)
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
        plotBfocus2(numberslog);

        mkl_free(numberslog);
        mkl_free(m_cosphy);
        mkl_free(m_sinphy);
}

void mainWidget::plotBfocus2(float *num)
{
    // 考虑log图像的显示；
    double tmp1;
    double tmp2;
    tmp1 = ui->spinBox_3->text().toDouble();   //添加数据
    tmp2 = ui->spinBox_4->text().toDouble();   //添加数据
    if (tmp2 > tmp1)
    {

        cv::Mat img(mainWidget::m_sampleLength/2, 800, CV_32F);

            for (int j = 0; j < 800; j++)
            {
                for (int i = 0; i < mainWidget::m_sampleLength/2; i++)
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

            img = img * 255;
            if(mainWidget::coarsemoveflag == true)
            {
//                float I3 = 0;
//                float I4 = 0;
                float sumline = 0;

                //emit sendrobotarm();
                if (mainWidget::moveloopidx == 1)
                {
                    rs_get_current_waypoint(mainWidget::pri_rshd_send, &waypoints);
                    mainWidget::z1 = waypoints.cartPos.position.z + 0.008;
                    mainWidget::zposition = mainWidget::z1;
                    //qDebug()<<"robotthread"<< mainWidget::z1 * 1000;
                    ui->coarsefButton->clicked(); 
                    qDebug()<<"moveloopidx"<<moveloopidx<<mainWidget::zposition;

                }
                if (mainWidget::moveloopidx == 2)
                {
                    mainWidget::z2 = mainWidget::z1 - 0.016;
                    //qDebug()<<"moveloop2"<< mainWidget::z2 * 1000;
                    mainWidget::zposition = mainWidget::z2;
                    ui->coarsefButton->clicked();
                    qDebug()<<"moveloopidx"<<moveloopidx<<mainWidget::zposition;
                }

                if(mainWidget::moveloopidx == 3)
                {
                    mainWidget::z3 = mainWidget::z1 + 0.618 * (mainWidget::z2 - mainWidget::z1);
                    mainWidget::zposition = mainWidget::z3;
                    ui->coarsefButton->clicked();
                    qDebug()<<"moveloopidx"<<moveloopidx<<mainWidget::zposition;
                }
                if(mainWidget::moveloopidx == 4)
                {
                    for(int j = 300; j < 500; j++)
                    {
                        for(int i = 6; i < 575; i++)
                        {
                            sumline = img.at<float>(i, j) + sumline;
                        }
                    }
                    mainWidget::I3 = sumline;
                    mainWidget::z4 = mainWidget::z2 + 0.618 * (mainWidget::z1 - mainWidget::z2);
                    mainWidget::zposition = mainWidget::z4;
                    ui->coarsefButton->clicked();
                    qDebug()<<"moveloopidx"<<moveloopidx<<mainWidget::zposition;
                }
                if(mainWidget::moveloopidx > 4 && mainWidget::moveloopidx < 14)
                {
                    for(int j = 300; j < 500; j++)
                    {
                        for(int i = 6; i < 575; i++)
                        {
                            sumline = img.at<float>(i, j) + sumline;
                        }
                    }

                    if(true)
                    {
                        if(mainWidget::moveloopidx % 2 ==1)
                        {
                            mainWidget::I4 = sumline;

                            // 用来更新位置
                            if((mainWidget::I3 > mainWidget::I4))
                            {

                                mainWidget::z1 = mainWidget::z4;
                                mainWidget::z2 = mainWidget::z2;
                                mainWidget::z3 = mainWidget::z1 + 0.618 * (mainWidget::z2 - mainWidget::z1);
                                mainWidget::zposition = mainWidget::z3;
                                ui->coarsefButton->clicked();
                                qDebug()<<"moveloopidx"<<moveloopidx<<mainWidget::zposition;
                            }

                            if((mainWidget::I3 < mainWidget::I4))
                            {
                                mainWidget::z2 = mainWidget::z3;
                                mainWidget::z1 = mainWidget::z1;
                                mainWidget::z3 = mainWidget::z1 + 0.618 * (mainWidget::z2 - mainWidget::z1);
                                mainWidget::zposition = mainWidget::z3;
                                ui->coarsefButton->clicked();
                                qDebug()<<"moveloopidx"<<moveloopidx<<mainWidget::zposition;
                            }
                        }
                        if(mainWidget::moveloopidx % 2 ==0)
                        {
                             mainWidget::I3 = sumline;
                             mainWidget::z4 = mainWidget::z2 + 0.618 * (mainWidget::z1 - mainWidget::z2);
                             mainWidget::zposition = mainWidget::z4;
                             ui->coarsefButton->clicked();
                             qDebug()<<"moveloopidx"<<moveloopidx<<mainWidget::zposition;
                        }

                    }

                }

//                if((mainWidget::moveloopidx > 4 && abs(mainWidget::z4 - mainWidget::z3) < 0.0002) || mainWidget::moveloopidx > 10)
//                {
//                    mainWidget::coarsemoveflag = false;
//                }

             }
//                else
//                {
//                    mainWidget::coarsemoveflag = false;
//                }

            mainWidget::moveloopidx = mainWidget::moveloopidx + 1;



            // 考虑血流成像的C++程序的写作；
            // 先不封装成函数
            cv::Mat img2(mainWidget::m_sampleLength/2, 720, CV_32F);
            img2 = img(Range(1, mainWidget::m_sampleLength/2), Range(1, 720));
            Mat Temp;

            img2.convertTo(Temp, CV_8UC1);


            QImage Qtemp = putImage(Temp);
            ui->label2D->setPixmap(QPixmap::fromImage(Qtemp.scaled(ui->label2D->size())));
}
            //connect(ui->fmove,SIGNAL(clicked()),rbThread, SLOT(fmoveto()));

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

QImage mainWidget::putImage(const Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS=1
    if (mat.type() == CV_8UC1)
    {

        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i = 0; i < 256; i++)
            colorTable.push_back(qRgb(i, i, i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    if (mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
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
    else if(mainWidget::scanMode == 23)//focus change扫描
    {
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        pDAUSB3020->Start2DscanRepeat();
        timerId3 = startTimer(700);
    }

    else if(mainWidget::scanMode == 24)//focus change扫描
    {
        pDAUSB3020->DisableDA();
        pDAUSB3020->ReleaseDA();
        pDAUSB3020->Start2DscanRepeat();
        timerId3 = startTimer(1000);
    }

}

void mainWidget::on_stopButton_clicked()
{
    ui->startButton->setEnabled(TRUE);
    ui->startButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/start.png);}");
    ui->stopButton->setEnabled(FALSE);
    ui->stopButton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/stopFinish.png);}");
    mainWidget::captureflag = 0;

    if (mainWidget::scanMode == 1)
    {
        killTimer(timerId1);
    }
    else if (mainWidget::scanMode == 2 )
    {
        killTimer(timerId2);

    }
    else if (mainWidget::scanMode == 10)
    {
        killTimer(timerId3);
    }
    else if (mainWidget::scanMode == 3)
    {
        killTimer(timerId2);
    }
    else if (mainWidget::scanMode == 32)
    {
        killTimer(timerId2);
    }
    else if (mainWidget::scanMode == 22)
    {
        killTimer(timerId3);
    }
    else if (mainWidget::scanMode == 23)
    {
        killTimer(timerId3);
    }
    else if (mainWidget::scanMode == 24)
    {
        killTimer(timerId3);
    }

    firstThread->quit();
    firstThread->wait();

    pDAUSB3020->DisableDA();
    pDAUSB3020->ReleaseDA();
    pDAUSB3020->StopScan();
    ui->connectBotton->setEnabled(TRUE);
    ui->connectBotton->setStyleSheet("QPushButton{border-image:url(:/new/prefix1/connect.png);}");
    return;

}



//void mainWidget::on_DAtest_clicked()
//{
//    OCTplots();
//}

void mainWidget::on_resetaxis_clicked()
{
    ui->cosplot->xAxis->setRange(0,mainWidget::m_sampleLength);
    ui->cosplot->yAxis->setRange(-2048,2048);
    //ui->cosplot->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);


    //ui->fftplot->addGraph();

    //设置坐标轴标签名称
    //ui->fftplot->xAxis->setLabel("x");
    //ui->fftplot->yAxis->setLabel("f");

    //设置坐标轴显示范围,否则我们只能看到默认的范围
    ui->fftplot->xAxis->setRange(1,mainWidget::m_sampleLength/2);
    ui->fftplot->yAxis->setRange(20,120);
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


void mainWidget::on_saveButton_clicked()
{
    QMessageBox *msgBox = new QMessageBox();
    msgBox->setStyleSheet("background-color:white");
    int ret = QMessageBox::question(this, "question",
             "你要保存的数据内容吗???",
              QMessageBox::Save|QMessageBox::Cancel,
              QMessageBox::Cancel);
    if(ret == QMessageBox::Save)
    {
        saveclicked();
        QMessageBox::information(this, "information", "恭喜你保存成功了");
    }
    else if(ret == QMessageBox::Cancel)
    {
        QMessageBox::warning(this, "warning", "你放弃了保存!");
        return;
    }
}


void mainWidget::on_textEdit_textChanged()
{
    ui->textEdit->moveCursor(QTextCursor::End);
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



void mainWidget::on_pb_login_clicked()
{
    if(Robot_Login() == true)
    {
        qDebug() << "login successfully";
        ui->textEdit->append("连接机械臂suc ");
        ui->pb_login->setEnabled(FALSE);
        ui->pb_login->setStyleSheet("QPushButton {background-color:rgb(25,25,25);\
                                    font: 75 11pt;\
                                    color:rgb(240,240,240);\
                                    border-radius:4px; height:25px;\
                                    width:100;\
                                    font-weight: bold}\
                                    QPushButton:hover{background-color:rgb(35,35,35);}");
        ui->logoutButton->setEnabled(TRUE);
        ui->logoutButton->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                        font: 75 11pt;\
                                        color:rgb(240,240,240);\
                                        border-radius:4px; height:25px;\
                                        width:100;\
                                        font-weight: bold}\
                                        QPushButton:hover{background-color:rgb(35,35,35);}");
    }
    else
    {
        ui->textEdit->append("连接机械臂fail ");
    }
}

void mainWidget::on_logoutButton_clicked()
{
    timer->stop();
    this->logout(mainWidget::pri_rshd_send);
    ui->textEdit->append("机械臂已断开");
    ui->pb_login->setEnabled(TRUE);
    ui->pb_login->setStyleSheet("QPushButton {background-color:rgb(55,55,55);\
                                font: 75 11pt;\
                                color:rgb(240,240,240);\
                                border-radius:4px; height:25px;\
                                width:100;\
                                font-weight: bold}\
                                QPushButton:hover{background-color:rgb(35,35,35);}");
    ui->logoutButton->setEnabled(FALSE);
    ui->logoutButton->setStyleSheet("QPushButton {background-color:rgb(25,25,25);\
                                    font: 75 11pt;\
                                    color:rgb(240,240,240);\
                                    border-radius:4px; height:25px;\
                                    width:100;\
                                    font-weight: bold}\
                                    QPushButton:hover{background-color:rgb(35,35,35);}");
    secondThread->quit();
    secondThread->wait();
}

void mainWidget::on_saveData_clicked()
{
    double tempx;
    double tempy;
    double tempz;
    double temprx;
    double tempry;
    double temprz;
    double maxvelocity;
    double maxacclrate;

    tempx= ui->xvalue->text().toDouble();
    tempy = ui->yvalue->text().toDouble();
    tempz = ui->zvalue->text().toDouble();
    temprx = ui->rxvalue->text().toDouble();
    tempry = ui->ryvalue->text().toDouble();
    temprz = ui->rzvalue->text().toDouble();
    maxvelocity = ui->maxVelocity->text().toDouble();
    maxacclrate = ui->maxAcc->text().toDouble();
    if(maxvelocity > 50 || maxvelocity < 0 || maxacclrate > 50 || maxacclrate < 0)
    {
        QMessageBox msgBox;
        msgBox.critical(nullptr, tr("提示"), tr("加速度，速度需要在5 - 50°之间 "));
        maxvelocity = 30;
        maxacclrate = 30;
        ui->maxVelocity->clear();//输入错误则清空，变为初始值
        ui->maxAcc->clear();
    }
    //接口调用: 初始化运动属性
    rs_init_global_move_profile(mainWidget::pri_rshd_send);
    //接口调用: 设置关节型运动的最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = maxacclrate / 180.0*M_PI;
    jointMaxAcc.jointPara[1] = maxacclrate / 180.0*M_PI;
    jointMaxAcc.jointPara[2] = maxacclrate / 180.0*M_PI;
    jointMaxAcc.jointPara[3] = maxacclrate / 180.0*M_PI;
    jointMaxAcc.jointPara[4] = maxacclrate / 180.0*M_PI;
    jointMaxAcc.jointPara[5] = maxacclrate / 180.0*M_PI;   //接口要求单位是弧度
    rs_set_global_joint_maxacc(mainWidget::pri_rshd_send, &jointMaxAcc);

    //接口调用: 设置关节型运动的最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = maxvelocity / 180.0*M_PI;
    jointMaxVelc.jointPara[1] = maxvelocity / 180.0*M_PI;
    jointMaxVelc.jointPara[2] = maxvelocity / 180.0*M_PI;
    jointMaxVelc.jointPara[3] = maxvelocity / 180.0*M_PI;
    jointMaxVelc.jointPara[4] = maxvelocity / 180.0*M_PI;
    jointMaxVelc.jointPara[5] = maxvelocity / 180.0*M_PI;   //接口要求单位是弧度
    rs_set_global_joint_maxvelc(mainWidget::pri_rshd_send, &jointMaxVelc);

    if(mainWidget::tooltype == 0)
    {
        temprx = temprx / 180 * M_PI;
        tempry = tempry / 180 * M_PI;
        temprz = temprz / 180 * M_PI;
    }
    if(mainWidget::tooltype == 1)
    {
        qDebug()<<"进入工具";
        // 基座标系转用户坐标系
        current_Coord.coordType = BaseCoordinate;
        Pos pos_onuser;
        Ori ori_onuser;
        Rpy rpy;

        // 写入一个Pos和Ori；
        Pos postarget;
        postarget.x = tempx;
        postarget.y = tempy;
        postarget.z = tempz;

        Rpy rpytarget;
        rpytarget.rx = temprx / 180 * M_PI;
        rpytarget.ry = tempry/ 180 * M_PI;
        rpytarget.rz = temprz/ 180 * M_PI;

        Ori oritarget;
        rs_rpy_to_quaternion(mainWidget::pri_rshd_send, &rpytarget, &oritarget);

        rs_user_to_base(mainWidget::pri_rshd_send, &postarget, &oritarget, &current_Coord, &mainWidget::tool, &pos_onuser, &ori_onuser);

        rs_quaternion_to_rpy(mainWidget::pri_rshd_send, &ori_onuser, &rpy);

        tempx = pos_onuser.x;
        tempy = pos_onuser.y;
        tempz = pos_onuser.z;
        temprx = rpy.rx;
        tempry = rpy.ry;
        temprz = rpy.rz;
    }
    mainWidget::x = tempx;
    mainWidget::y = tempy;
    mainWidget::z = tempz;
    mainWidget::rx = temprx;
    mainWidget::ry = tempry;
    mainWidget::rz = temprz;
    ui->textEdit->append("目标位姿已写入 ");
}

void mainWidget::recieveIK(int ret)
{
    if(ret != RS_SUCC)
    {
        QMessageBox msgBox;
        msgBox.critical(nullptr, tr("提示"), tr("逆解错误 "));
    }
}

void mainWidget::on_connectcameraButton_clicked()
{
    // get camera pointer and connect a valid camera
    mainWidget::camera = cs::getCameraPtr();
    mainWidget::cstatus = mainWidget::camera->connect();
    if(mainWidget::cstatus != SUCCESS)
    {
        ui->textEdit->append("camera connect failed:" + QVariant(mainWidget::ret).toString() + "\n" );
    }
    else
    {
        ui->textEdit->append("camera connect succesed!\n");
    }

    // get  informations of camera
    CameraInfo info;
    mainWidget::ret = mainWidget::camera->getInfo(info);
    if(mainWidget::ret != SUCCESS)
    {
        ui->textEdit->append("camera get info failed:" + QVariant(mainWidget::ret).toString() + "\n");
    }

    // display informations of camera
    ui->textEdit->append("name:" + QVariant(info.name).toString());
    ui->textEdit->append("serial:" + QVariant(info.serial).toString());
    ui->textEdit->append("unique id:" + QVariant(info.uniqueId).toString());
    ui->textEdit->append("firmware version:" + QVariant(info.firmwareVersion).toString());
    ui->textEdit->append("algorithm version:" + QVariant(info.algorithmVersion).toString());
    ui->textEdit->append("\n");
}

void mainWidget::on_opencameraButton_clicked()
{
    if(mainWidget::cstatus == SUCCESS)
    {
        qDebug() << "open clicked";
        thirdThread->start();//进入线程

        if(ui->rgbCheckBox->isChecked()==true)//判断是否勾选rgb
        {
            mainWidget::rgbFlag = 1;
            MyCamThread->startRgbStream();
        }
        if(ui->depthCheckBox->isChecked()==true)//判断是否勾选深度
        {
            mainWidget::depthFlag = 1;
            MyCamThread->startDepthStream();
        }

        MyCamThread->setparams();

        QEventLoop eventloop;
        QTimer::singleShot(50, &eventloop, SLOT(quit())); //wait 0.8s
        eventloop.exec();

        fps_timer.start();
    }
}
//接受rgb图片并显示
void mainWidget::receiveimage(cv::Mat rgbimage)
{
    if (rgbimage.empty())
    {
        qDebug() << "rgb image is empty";
     }
    else
    {
    qDebug() << "start show image";
    QImage rgb_QImage = QImage((const unsigned char*)(rgbimage).data, rgbimage.cols, rgbimage.rows, QImage::Format_RGB888);
    //QImage rgb_QImage = putImage(rgbimage);
    int width = ui->rgblabel->width();
    int height = ui->rgblabel->height();
    QPixmap pixmap = QPixmap::fromImage(rgb_QImage);
    QPixmap fitpixmap = pixmap.scaled(width,height,Qt::KeepAspectRatio);
    ui->rgblabel->setPixmap(fitpixmap);
    }
}
//接受depth图片并显示
void mainWidget::receiveimage2(cv::Mat depthimage)
{

    //qDebug() << "receiveimage2";

    //qDebug() << depthimage.at<cv::Vec3b>(100, 100)[1];
    if (depthimage.empty())
    {
        qDebug() << "rgb image is empty";
    }

    else
    {
        //qDebug() << "start show image";
        QImage depth_QImage = QImage((unsigned char*)(depthimage).data, depthimage.cols, depthimage.rows, QImage::Format_RGB888);
        int width = ui->depthlabel->width();
        int height = ui->depthlabel->height();
        QPixmap pixmapdepth = QPixmap::fromImage(depth_QImage);
        QPixmap fitpixmapdpth = pixmapdepth.scaled(width,height,Qt::KeepAspectRatio);
        ui->depthlabel->setPixmap(fitpixmapdpth);
    }
}
void mainWidget::receiveimage3(cv::Mat rgbimage, cv::Mat depthimage)
{
    if (rgbimage.empty())
    {
        qDebug() << "rgb image is empty";
     }
    else
    {
    //qDebug() << "start show image";
    QImage rgb_QImage = QImage((const unsigned char*)(rgbimage).data, rgbimage.cols, rgbimage.rows, QImage::Format_RGB888);
    int width = ui->rgblabel->width();
    int height = ui->rgblabel->height();
    QPixmap pixmap = QPixmap::fromImage(rgb_QImage);
    QPixmap fitpixmap = pixmap.scaled(width,height,Qt::KeepAspectRatio);
    ui->rgblabel->setPixmap(fitpixmap);

    }

    if (depthimage.empty())
    {
        qDebug() << "rgb image is empty";
    }

    else
    {
        //qDebug() << "start show image";
        QImage depth_QImage = QImage((unsigned char*)(depthimage).data, depthimage.cols, depthimage.rows, QImage::Format_RGB888);
        int width = ui->depthlabel->width();
        int height = ui->depthlabel->height();
        QPixmap pixmapdepth = QPixmap::fromImage(depth_QImage);
        QPixmap fitpixmapdpth = pixmapdepth.scaled(width,height,Qt::KeepAspectRatio);
        ui->depthlabel->setPixmap(fitpixmapdpth);
    }
}

void mainWidget::on_closecameraButton_clicked()
{
    qDebug()<<"close clicked";
    fps_timer.stop();//关闭时钟
    thirdThread->quit();
    thirdThread->wait();
    // 关闭rgb流的采集
    qDebug()<<"close clicked2";
    mainWidget::ret = mainWidget::camera->stopStream(STREAM_TYPE_RGB);
    qDebug()<<"close clicked3";
    if(mainWidget::ret != SUCCESS)
    {
        ui->textEdit->append("camera stop rgb stream failed!"+ QVariant(mainWidget::ret).toString());
    }
    else
    {
        ui->textEdit->append("RGB stream has been disconnected!\n");
    }

    // 关闭深度流的采集
    mainWidget::ret = mainWidget::camera->stopStream(STREAM_TYPE_DEPTH);
    qDebug()<<"close clicked4";
    if(mainWidget::ret != SUCCESS)
    {
        ui->textEdit->append("camera stop depth stream failed!"+ QVariant(mainWidget::ret).toString());
    }
    else
    {
        ui->textEdit->append("depth stream has been disconnected!\n");
    }

    // disconnect camera
    mainWidget::ret = mainWidget::camera->disconnect();
    qDebug()<<"close clicked5";
    if(ret != SUCCESS)
    {
        ui->textEdit->append("camera disconnect failed:" + QVariant(mainWidget::ret).toString() + "\n");
    }
    else
    {
        ui->textEdit->append("camera has been disconnected!\n");
    }

    qDebug()<<"close clicked6";

}

void mainWidget::on_identifylabel_clicked()
{
     ui->textEdit->append("打开识别功能 ");
    mainWidget::identifyFlag = 1;
    if(ui->colorcombo->currentText() == tr("红色"))
    {
        mainWidget::labelcolor = 1;
        ui->textEdit->append("识别红色 ");
    }
    if(ui->colorcombo->currentText() == tr("蓝色"))
    {
        mainWidget::labelcolor = 2;
        ui->textEdit->append("识别蓝色 ");
    }
    if(ui->colorcombo->currentText() == tr("绿色"))
    {
        mainWidget::labelcolor = 3;
        ui->textEdit->append("识别绿色 ");
    }

}

void mainWidget::on_closeidentify_clicked()
{
    ui->textEdit->append("关闭识别功能 ");

    mainWidget::identifyFlag = 0;

}


void mainWidget::on_paramsButton_clicked()
{
    mainWidget::exposure = ui->expotext->text().toFloat();
    if(mainWidget::exposure > 40000 || mainWidget::exposure < 3000)
    {
        QMessageBox msgBox;
        msgBox.critical(nullptr, tr("提示"), tr("曝光时间需要在3000-40000之间! "));
        mainWidget::exposure = 3000;
        ui->expotext->clear();//输入错误则清空，变为初始值
    }
    mainWidget::gain = ui->gaintext->text().toFloat();
    if(mainWidget::gain > 10 || mainWidget::gain < 1)
    {
        QMessageBox msgBox;
        msgBox.critical(nullptr, tr("提示"), tr("增益参数需要在1-10之间! "));
        mainWidget::exposure = 1;
        ui->expotext->clear();
    }
    //设置最小范围参数
    mainWidget::minRange = ui->minrange->text().toInt();
    //设置最大范围参数
    mainWidget::maxRange = ui->maxrange->text().toInt();

    if(mainWidget::minRange < 50 ||  mainWidget::minRange > mainWidget::maxRange || mainWidget::maxRange > 3000)
    {
        QMessageBox msgBox;
        msgBox.critical(nullptr, tr("提示"), tr("最小范围参数需要在50-3000之间! "));
        mainWidget::minRange = 50;
        mainWidget::maxRange = 3000;
        ui->minrange->clear();
        ui->maxrange->clear();
    }
    MyCamThread->setparams();

}

void mainWidget::on_coordtransButton_clicked()
{
    mainWidget::rgbX = ui->rgbx->text().toInt();

    mainWidget::rgbY = ui->rgby->text().toInt();

}

void mainWidget::on_transfercoordButton_clicked()
{
    ui->textEdit->append("ransfercoordButton_clicked ");
    ui->depthx->clear();//先清空显示窗口再显示，可以达到实时刷新显示的目的
    ui->depthx->setText(QVariant(mainWidget::depthX).toString());
    ui->depthy->clear();//先清空显示窗口再显示，可以达到实时刷新显示的目的
    ui->depthy->setText(QVariant(mainWidget::depthY).toString());

}



/*
	* Date: 2022.3.24
	* Details: 图像获取和处理控制
	* Remarks: 利用多线程异步处理图像的获取与处理步骤，提高cpu的利用率，提高程序运行速度，图像的获取与处理控制采用生产者和消费者的方法
*/

#include "../header/ImageProcess.h"
#include "../DaHengCamera/DaHengCamera.h"
#include<sys/timeb.h>

// 模式设置
#define USING_CAMERA			// 使用大恒相机获取图像
// #define USING_VIDEO				// 使用视频进行调试
// #define DEBUG_ARMOR				// 利用装甲板视频进行调试
// #define DEBUG_BUFF			// 利用能量机关视频进行调试
// #define ANGLESOLVER		        // 是否进行角度解算

#define BUFFER 5		// 线程间相机采集最高超过处理的帧数
ImageDate Image[BUFFER];
static volatile unsigned int proIdx = 0;    // 生产id
static volatile unsigned int consIdx = 0;   // 消费id
double run_time = 0;        // 记录时间

// 初始化数据
// int targetNum = 2;			// 设置需要击打的装甲板目标数字
// Color ENEMYCOLOR = RED;	// 设置敌方战车灯条颜色
double fps;

// 导入装甲板识别类
ArmorDetector detector;
// 导入角度解算类
AngleSolver angleSolver;

// 无参数构造函数
ImageProcess::ImageProcess() {
    proIdx = 0;
    consIdx = 0;
}

// 获取图像函数
void ImageProcess::ImageProducter() {
#ifdef DEBUG_ARMOR		// 利用装甲板视频进行调试
    VideoCapture cap(Armor_videopath);
#endif
#ifdef DEBUG_BUFF		// 利用能量机关视频进行调试
    VideoCapture cap(Buff_videopath);
#endif

#ifdef USING_CAMERA
    // 使用大恒相机获取图像进行调试
    DaHengCamera CameraTure;
    // 链接设备
    CameraTure.StartDevice(1);
    // 设置分辨率(设置分辨率要在摄像头开始采集之前)
    CameraTure.SetResolution(2,2);
    // 控制设备开采
    CameraTure.SetStreamOn();
    // 设置曝光值
    CameraTure.SetExposureTime(GXExpTime);
    // 设置曝光增益
    CameraTure.SetGAIN(3,GXGain);
    // 关闭自动白平衡
    CameraTure.Set_BALANCE_AUTO(0);
#endif // USING_CAMERA

    // 生产部分循环
    do {
        while (proIdx - consIdx >= BUFFER);
        // 挂资源锁
        reciveRes.lock();
        // 解锁
        reciveRes.unlock();
        cout << "进入生产!" << endl;
        ImageDate Src;
        // 使用收数资源
        // 挂资源锁
        reciveRes.lock();
        Src.ReciveStm32.BeginToNowTime = getStm32.BeginToNowTime;
        Src.ReciveStm32.pitch = getStm32.pitch;
        Src.ReciveStm32.yaw = getStm32.yaw;
        Src.ReciveStm32.ShootSpeed = getStm32.ShootSpeed;
        // cout<<"得到收数时间"<<getStm32.BeginToNowTime<<endl;
        // cout<<"收数pitch:"<<Src.ReciveStm32 .pitch<<"    yaw:"<<Src.ReciveStm32.yaw<<"   射速:"<<getStm32.ShootSpeed<<endl<<endl;
        // 解锁
        reciveRes.unlock();

#ifdef DEBUG_ARMOR		// 利用装甲板视频进行调试
        cap >> Src.SrcImage;
        Src.mode = VIDEO;
#endif
#ifdef DEBUG_BUFF		// 利用能量机关视频进行调试
        cap >> Src.SrcImage;
        Src.mode = BUFF;
#endif

#ifdef USING_CAMERA
        // 使用大恒相机获取图像
        CameraTure.GetMat(Src.SrcImage);
        if (Src.SrcImage.cols == 0) {
            cout << "丢帧!" << endl;
            continue;
        }
        Src.mode = DAHENG;
#endif // USING_CAMERA

        Image[proIdx % BUFFER] = Src;       // 存入图像

        proIdx++;
        cout << "生产完成: " << proIdx - 1 << endl;

    } while (bool_Run);

#ifdef DEBUG_ARMOR
    cap.release();
#endif
#ifdef DEBUG_BUFF
    cap.release();
#endif
}

// 消费图像函数
void ImageProcess::ImageConsumer() {
    // 导入数字识别模板
    detector.loadSVM("../123svm.xml");

#ifdef USING_CAMERA
    #ifdef ANGLESOLVER
    // 使用相机
    // 设置基本参数
    // angleSolver.setCameraParam("camera_params.xml", 1);
    angleSolver.setArmorSize(SMALL_ARMOR, 135, 125);
    angleSolver.setArmorSize(BIG_ARMOR, 230, 127);
#endif
#endif

    double t, t1;
    do {
        while (consIdx >= proIdx);
        t = static_cast<double>(getTickCount());     // 记录时间
        int ImageIndex = proIdx - 1;              // 获取最新图像的index

        if (Image[(ImageIndex) % BUFFER].mode != BUFF) {        // 自瞄(非BUFF模式)
            // 设置敌方战车灯条颜色
            detector.setEnemyColor(ENEMYCOLOR);
            if (Image[(ImageIndex) % BUFFER].mode == VIDEO) {   // 读取视频模式
                Image[(ImageIndex) % BUFFER].SrcImage.copyTo(src);      // 将获取到的图像copy to src中
                if (src.empty())
                    break;
                detector.setImg(src);       // 装甲板识别图像预处理
                detector.run(src);          // 装甲板识别综合函数
            }
            else if (Image[(ImageIndex) % BUFFER].mode == DAHENG) {     // 读取大恒相机模式
                Image[(ImageIndex) % BUFFER].SrcImage.copyTo(src);      // 将获取到的图像copy to src中
                if (src.empty())
                    break;
                detector.setImg(src);       // 装甲板识别图像预处理
                detector.run(src);          // 装甲板识别综合函数
            }

            // 获取装甲板识别后的相关数据 用于后续操作以及DEBUG信息输出
            vector<Point2f> contourPoints;
            ArmorType type;
            Point2f centerPoint;
            ArmorBox BestArmor;
            pattern Car_model;
            if (detector.isFoundArmor())
                detector.getTargetInfo(BestArmor, contourPoints, centerPoint, type, Car_model);

            // targetNum = 2;
            // 设置目标装甲板数字
            // detector.setTargetNum(targetNum);

            // 角度解算
#ifdef ANGLESOLVER
            double yaw = 0, pitch = 0, distance = 0;
            if (Car_model == buffering) {       // 缓冲模式
                // 缓冲状态击打
                angleSolver.BufferSetFilter(BestArmor, Image[(ImageIndex) % BUFFER].ReciveStm32, pitch, yaw);
            }
            else if (Car_model == FirstFind || Car_model == Shoot) {        // 首次识别模式，连续射击模式
                // 正常射击状态
                angleSolver.getAngle(Car_model, BestArmor, Image[(ImageIndex) % BUFFER].ReciveStm32, contourPoints, centerPoint, type, yaw, pitch, distance);
            }

            // 发送数据给端口，为发送串口数据给下位机做准备
            ArmorToData(Car_model, pitch, yaw);
#endif // ANGLESOLVER

        }
        else {          // 能量机关(BUFF模式)

        }

        // FPS
        t1 = static_cast<double>((getTickCount() - t) / getTickFrequency());
        printf("Armor Detecting FPS: %f\n", 1 / t1);

#ifdef DEBUG_MODE
        //********************** DEGUG **********************//
        //装甲板检测识别调试参数是否输出
        //param:
        //		1.showSrcImg_ON,		  是否展示原图
        //		2.bool showSrcBinary_ON,  是否展示二值图
        //		3.bool showLights_ON,	  是否展示灯条图
        //		4.bool showArmors_ON,	  是否展示装甲板图
        //		5.bool textLights_ON,	  是否输出灯条信息
        //		6.bool textArmors_ON,	  是否输出装甲板信息
        //		7.bool textScores_ON	  是否输出打击度信息
        //					   1  2  3  4  5  6  7
        detector.showDebugInfo(1, 1, 1, 1, 0, 0, 0);

        char chKey = cv::waitKey(50);       // 修改waitKey获取的值可以控制视频读取的速率
        switch (chKey) {
            case 'b':           // 输入b/B可将敌方战车颜色设置为蓝色
            case 'B':
                ENEMYCOLOR = BLUE;
                break;
            case 'r':           // 输入r/R可将敌方战车颜色设置为红色
            case 'R':
                ENEMYCOLOR = RED;
                break;
            case 'q':           // 输入q/Q可退出程序循环
            case 'Q':
            case 27:
                bool_Run = false;
                break;
            default:
                break;
        }
#endif

        consIdx = ImageIndex + 1;
        cout << "消费完成" << endl;
    } while (bool_Run);
}

// 保存参数函数
void ImageProcess::SaveData() {
    // 文件读取
    // OpenCV中可以使用FileStorage对xml,yml等文件进行读写操作
    FileStorage fs_param(paramFileName, FileStorage::WRITE);    // 进行写操作
    if (!fs_param.isOpened()) {
        cout << "参数文件打开失败!" << endl;
        exit(1);
    }
}
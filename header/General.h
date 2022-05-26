#pragma once

#ifndef RM_GENERAL_H
#define RM_GENERAL_H

#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
// 互斥量类是一个同步原语，可以用来保护共享数据不被多个线程同时访问
#include<mutex>

#define DEBUG_MODE
#define RELEASE_MODE
#define PI 3.14159
#define IMSHOW
#define Serial_Path "/dev/485_USB"      // 串口路径
#define Serial_Baud 0                   // 串口波特率(0表示115200    1表示921600)

using namespace std;

// 外部变量
// extern pthread_mutex_t Globalmutex; // 由于图像更新导致的线程冲突
// extern pthread_cond_t GlobalCondCV; // 由于图像更新导致的线程冲突
// extern bool imageReadable;          // 由于图像更新导致的线程冲突
extern cv::Mat src;                    // 定义源图像src
extern string Armor_videopath;         // 装甲板测试视频路径
extern string Buff_videopath;          // 能量机关测试视频路径
extern bool bool_Run;		// 程序是否运行
extern mutex reciveRes;     //线程读取资源锁
extern string paramFileName;    // 参数读取路径

// 图像更新线程
// void* imageUpdatingThread(void* PARAM);

// 装甲板识别线程
// void* armorDetectingThread(void* PARAM);

// 战车击打模式
enum HitType
{
    SightType = 0,      // 自瞄模式
    EnergyType = 1      // 能量机关模式
};

// 是否发射子弹
enum BoolHit
{
    Hit_yes = 0,        // 发射子弹
    Hit_no = 1          // 不发射子弹
};

// 装甲板类型
enum ArmorType
{
    SMALL_ARMOR = 0,    // 小装甲板 步兵，英雄
    BIG_ARMOR = 1       // 大装甲板 哨兵
};

// 敌方战车颜色（灯条颜色） B蓝 G绿 R红
enum Color
{
    BLUE = 0,
    GREEN = 1,
    RED = 2
};
extern Color ENEMYCOLOR;        // 所需击打战车的灯条颜色

// 射击模式
enum pattern {
    FirstFind,       // 首次识别
    Shoot,           // 连续射击
    Stop,            // 非连续
    buffering        // 缓冲
};

// 上位机接收当前的yaw和pitch值
struct CarData {
    double pitch = 0;       // 获取战车当前的pitch值
    double yaw = 0;         // 获取战车当前的yaw值
    float ShootSpeed = 16;  // 子弹射速
    double BeginToNowTime = 0;      // 获取时间
    Color c = RED;
};

// 击打缓冲计算返回
typedef struct {
    float pitch;
    float yaw;
    float t;     // 击打弹道时间
    float angle;
} Angle_t;

// 读取图像模式
typedef enum {
    VIDEO,              // 视频读取
    DAHENG,             // 大恒相机
    BUFF                // 大符
} ImageGetMode;

// 采入图像
struct ImageDate {
    cv::Mat SrcImage;
    ImageGetMode mode;      // 读取图像模式
    CarData ReciveStm32;    // 下位机通过串口传输而来
};

extern CarData getStm32;    // 读取下位机发送的数据

// 获取两个点的距离
float getPointsDistance(const cv::Point2f& a, const cv::Point2f& b);

// 根据打击优先级增加装甲板的打击度
void setNumScore(const int& armorNum, const int& targetNum, float& armorScore);

//
void ArmorToData(pattern Car_model, double pitch, double yaw);

/* -------------------相机参数--------------------- */
// 相机参数读取目录
extern string CameraFileName;

//阈值
extern int GrayValue;
extern int RGrayWeightValue;
extern int BGrayWeightValue;

//hsv
extern int RLowH ;
extern int RHighH;
extern int RLowS ;
extern int RHighS ;
extern int RLowV ;
extern int RHighV ;

extern int BLowH ;
extern int BHighH;
extern int BLowS;
extern int BHighS;
extern int BLowV ;
extern int BHighV;

extern int V_ts;

//相机曝光
extern int GXExpTime ;             //大恒相机曝光值
extern int GXExpTimeValue;         //控制大恒相机曝光值动态变化
extern int GXGain ;                //大恒相机增益
extern int GXGainValue;            //控制大恒相机曝光增益值动态变化

#endif //RM_GENERAL_H

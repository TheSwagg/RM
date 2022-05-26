#include "../header/RemoteController.h"

// 全局收发数变量
RemoteController Send;
RemoteController Recive;

#define BUFFER_SIZE 1
SerialPort serial;
serial_receive_data rx_data;
serial_transmit_data tx_data;
volatile unsigned int SerialBuffer;

RemoteController::RemoteController() {
    BeginTime = (double)getTickCount();
}

// 从下位机接收数据
void RemoteController::paraGetCar() {
    bool mode = 0;
    float pitch = 0;
    float yaw = 0;
    float ShootSpeed = 0;
    int col = 0;

    while (1) {
        serial.read_data(&rx_data, mode, ShootSpeed, pitch, yaw, col);
        CarData GetCar;
        GetCar.pitch = pitch;
        GetCar.yaw = yaw;
        GetCar.ShootSpeed = ShootSpeed;
        double timeing = ((double)getTickCount() - BeginTime) / getTickFrequency();
        GetCar.BeginToNowTime = timeing * 1000.0;
        if (col == 0)
            GetCar.c = BLUE;
        else
            GetCar.c = RED;

        // 挂起资源锁
        reciveRes.lock();
        getStm32.pitch = GetCar.pitch;
        getStm32.yaw = GetCar.yaw;
        getStm32.ShootSpeed = GetCar.ShootSpeed;
        getStm32.BeginToNowTime = GetCar.BeginToNowTime;
        getStm32.c = GetCar.c;
        // 解锁
        reciveRes.unlock();
    }
}

// 上位机接收数据发送给下位机
void RemoteController::paraReceiver() {
    serial = SerialPort(Serial_Path, Serial_Baud);		// 打开并初始化串口
    while (1)
    {
        while (SerialBuffer >= BUFFER_SIZE);
        SerialBuffer = BUFFER_SIZE;
        serial.send_data(tx_data);
    }
    serial.close_port();
}

// 汇总（接收）角度解算的数据，为下一步发送数据到下位机做准备
void RemoteController::ArmorToData(pattern Car_model, double pitch, double yaw) {
    if (Car_model == Stop)
        tx_data.get_xy_data(0, 0);
    else
        tx_data.get_xy_data(int16_t(pitch * 32767 / 90), int16_t(yaw * 32767 / 90));
    SerialBuffer--;
}

// 获取时间
double RemoteController::getClock() {
    double timeing = ((double)getTickCount() - BeginTime) / getTickFrequency();
    return timeing * 1000.0;
}
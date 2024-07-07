/*
 * @author BusyBox
 * @date 2024/5/31
 * @version 1.0
 * @details 维特智能WIT私有协议
 * @git https://github.com/ExceptionQWQ/RobotDevKit
 */

#pragma once

#include <cstdint>
#include <memory>
#include <functional>
#include <string>
#include "string.h"
#include "rdk/core/transfer/io_stream.h"
#include "stm32f4xx_hal.h"

class WitPrivateProcotol
{
public:
    WitPrivateProcotol(std::shared_ptr<IOStream> io_stream);
    ~WitPrivateProcotol();

    /*
     * @brief 开始接收数据，阻塞函数，建议新开任务调用
     */
    void start();

    struct AngleInfo
    {
        double roll;
        double pitch;
        double yaw;
        int16_t version;
        uint32_t time_stamp;
    };

    struct TimeOutException
    {
        std::string error;
    };

    /*
     * @brief 获取角度信息
     * @return 角度信息，包含roll，pitch，yaw
     * @attention 当传感器超过一定时间没有返回数据时会产生超时异常
     */
    AngleInfo get_angle_info();

    /*
     * @brief 加速度校准必须将模块正面放置去校准，如果模块反面放置校准会导致加速度异常，从而导致角度异常
     */
    void acceleration_calibration();
    /*
     * @brief 角度参考是以传感器当前的实际位置，让xy轴的角度归零，做一个相对归零的操作
     */
    void angle_reference();
    /*
     * @brief z轴归零需要在6轴算法的前提下，算法切换可以在上位机配置界面修改，9轴设备下的9轴算法是绝对角度，不能归零
     */
    void set_z_axis_to_zero();

private:
    std::shared_ptr<IOStream> io_stream;

    bool b_is_terminate = false;

    AngleInfo angle_info = {0};

#pragma pack(push, 1)
    struct RecvTimeInfo
    {
        uint8_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minutes;
        uint8_t seconds;
        uint16_t milliseconds;
        uint8_t sum;
    };

    struct RecvAccelerationInfo
    {
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t temp; //温度
        uint8_t sum;
    };

    struct RecvAngularVelocityInfo
    {
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t voltage;
        uint8_t sum;
    };

    struct RecvAngleInfo
    {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        int16_t version;
        uint8_t sum;
    };

    struct RecvMagneticInfo
    {
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t temp;
        uint8_t sum;
    };

    struct RecvPortStateInfo
    {
        int16_t d0;
        int16_t d1;
        int16_t d2;
        int16_t d3;
        uint8_t sum;
    };

    struct RecvPressureHeightInfo
    {
        int32_t pressure;
        int32_t height;
        uint8_t sum;
    };

    struct RecvLongitudeAndLatitudeInfo
    {
        int32_t lon;
        int32_t lat;
        uint8_t sum;
    };

    struct RecvGPSInfo
    {
        int16_t height;
        int16_t yaw;
        int32_t ground_velocity;
        uint8_t sum;
    };

    struct RecvQuarternionInfo
    {
        int16_t q0;
        int16_t q1;
        int16_t q2;
        int16_t q3;
        uint8_t sum;
    };

    struct RecvGPSPositioningAccuracyInfo
    {
        int16_t sn; //卫星数
        int16_t pdop; //位置定位精度
        int16_t hdop; //水平定位精度
        int16_t vdop; //垂直定位精度
        uint8_t sum;
    };

    struct RecvReadRegisterInfo
    {
        uint16_t reg1;
        uint16_t reg2;
        uint16_t reg3;
        uint16_t reg4;
        uint8_t sum;
    };

    struct WriteRegCommand
    {
        uint8_t head0 = 0xFF;
        uint8_t head1 = 0xAA;
        uint8_t addr;
        int16_t data;
    };
#pragma pack(pop)

    enum class RecvDataType : uint8_t
    {
        Time = 0x50,
        Acceleration = 0x51,
        AngularVelocity = 0x52,
        Angle = 0x53,
        Magnetic = 0x54,
        PortState = 0x55,
        PressureHeight = 0x56,
        LongitudeAndLatitude = 0x57,
        GPS = 0x58,
        Quarternion = 0x59,
        GPSPositioningAccuracy = 0x5A,
        ReadRegister = 0x5F,
    };

    void read_sensor_thread(WitPrivateProcotol* wit);

    void terminate_read_sensor_thread();
    bool is_terminate();

    std::size_t recv_frame(uint8_t* buff, std::size_t buff_len);

    uint8_t checksum(uint8_t* buff);
    void save_angle_info(uint8_t* buff);

    void unlock();
    void save();

};
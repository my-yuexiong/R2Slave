/*
 * @author BusyBox
 * @date 2024/5/10
 * @version 1.0
 * @git https://github.com/ExceptionQWQ/RobotDevKit
 */

#pragma once

#include <memory>
#include <cmath>
#include "rdk/core/motor/motor.h"


/*
     * @brief 麦克纳姆轮
     *
     *  |-----------------------|
     *  | motor1(A)   motor2(B) |
     *  |                       |
     *  |                       |
     *  |                       |
     *  |                       |
     *  |                       |
     *  |                       |
     *  |                       |
     *  |                       |
     *  | motor4(B)   motor3(A) |
     *  |-----------------------|
     *
     *  小车前方为x轴，左边为y轴
     *  电机转向应该满足当所有电机正转时，机器人沿x正方向运动
     *
     */
class MecanumMotion
{
public:
    MecanumMotion(std::shared_ptr<Motor> motor1, std::shared_ptr<Motor> motor2, std::shared_ptr<Motor> motor3, std::shared_ptr<Motor> motor4);
    ~MecanumMotion();

    /*
     * @brief 每隔1ms调用一次tick，会自动调用4个motor的tick函数
     */
    void tick();
    /*
     * @brief 设置轮子半径
     * @param wheel_radius 轮子半径(mm)
     */
    void set_wheel_radius(double wheel_radius);
    /*
     * @brief 清除缓存区的速度
     */
    void clear();
    /*
     * @brief 添加x轴速度
     * @param speed (mm/s)
     */
    void add_x_speed(double speed);
    /*
     * @brief 添加y轴速度
     * @param speed (mm/s)
     */
    void add_y_speed(double speed);
    /*
     * @brief 添加z轴速度
     * @param speed
     */
    void add_z_speed(double speed);
    /*
     * @brief 将缓冲中的速度写入电机
     */
    void commit();

private:
    std::shared_ptr<Motor> motor1;
    std::shared_ptr<Motor> motor2;
    std::shared_ptr<Motor> motor3;
    std::shared_ptr<Motor> motor4;

    double motor1_rpm = 0;
    double motor2_rpm = 0;
    double motor3_rpm = 0;
    double motor4_rpm = 0;

    double sqrt2 = std::sqrt(2);
    double pi = 3.1415926;

    double wheel_radius = 76;

};
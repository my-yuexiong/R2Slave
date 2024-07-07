/*
 * @author BusyBox
 * @date 2024/4/27
 * @version 1.0
 * @git https://github.com/ExceptionQWQ/RobotDevKit
 */

#include "rdk/core/motor/dji/m3508_motor.h"

M3508Motor::M3508Motor(std::shared_ptr<C6xxController> c6xx_controller, int id, Mode mode) : DjiMotor(c6xx_controller, id, mode)
{
    DjiMotor::set_speed_pid(16, 1.2, 20);
    if (mode == Mode::SPEED_POS) {
        DjiMotor::set_target_rpm(3000); //转速设置过大会丢步
        DjiMotor::set_pos_pid(0.12, 0, 0.12);

    }
    set_max_output_current(10);
}

M3508Motor::~M3508Motor()
{

}

/*
 * @brief 设置目标转速
 * @param rpm 转速
 */
void M3508Motor::set_target_rpm(double rpm)
{
    DjiMotor::set_target_rpm(rpm * reduction_ratio);
}

/*
 * @brief 设置目标位置
 * @param target_pos 目标位置，角度
 */
void M3508Motor::set_target_pos(double target_pos)
{
    DjiMotor::set_target_pos(target_pos * reduction_ratio / 360.0 * 8192.0);
}

/*
 * @brief 获取转速
 * @return rpm
 */
double M3508Motor::get_rpm()
{
    return DjiMotor::get_rpm() / reduction_ratio;
}

/*
 * @brief 获取位置
 * @return 角度
 */
double M3508Motor::get_pos()
{
    return DjiMotor::get_pos() / 8192.0 * 360.0 / reduction_ratio;
}

/*
 * @brief 设置电机最大输出电流
 * @param current 电流(A)
 */
void M3508Motor::set_max_output_current(double current)
{
    if (current < 0) current = -current;
    max_output_current = current / 20.0 * 32768;
    set_speed_pid_output_range(-max_output_current, max_output_current);
}
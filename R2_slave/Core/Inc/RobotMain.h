/*
 * @author BusyBox
 * @date 2024/4/27
 * @version 1.0
 */

#pragma once

#include "usart.h"
#include "can.h"
#include "tim.h"

#ifdef __cplusplus

#include <functional>
#include <memory>

#include "rdk/core/transfer/serial_port.h"
#include "rdk/core/transfer/reliable_binary_transfer.h"
#include "rdk/core/motor/dji/c6xx_controller.h"
#include "rdk/core/motor/dji/m2006_motor.h"
#include "rdk/core/motor/dji/m3508_motor.h"
#include "rdk/core/underpan/omnidirectional_motion.h"
#include "rdk/core/remote/dr16.h"
#include "rdk/core/servo/feetech/feetech_protocol.h"
#include "rdk/core/servo/feetech/feetech_SCS.h"
#include "rdk/core/servo/servo.h"
#include "rdk/core/servo/huaner/huaner_protocol.h"
#include "rdk/core/screw/Screw.h"

extern std::shared_ptr<SerialPort> serial_port;
extern std::shared_ptr<SerialPort> master_port;
//extern std::shared_ptr<SerialPort>ftSerial;
extern std::shared_ptr<ReliableBinaryTransfer> transfer;
extern std::shared_ptr<C6xxController> c6xx_controller1;
//extern std::shared_ptr<C6xxController> c6xx_controller2;
extern std::shared_ptr<M3508Motor> m3508_motor1;
extern std::shared_ptr<M3508Motor> m3508_motor2;
extern std::shared_ptr<M3508Motor> m3508_motor3;
extern std::shared_ptr<M3508Motor> m3508_motor4;
extern std::shared_ptr<OmnidirectionalMotion> motion;
extern std::shared_ptr<HuanerProtocol> ServoControl;
extern std::shared_ptr<Screw> MyScrew;

enum class MasterCmdType : uint8_t
{
    Ping = 0x01, //通信测试
    Motion = 0x02,  //控制底盘移动
    Servo = 0x03,  //控制舵机
    Screw = 0x04  //控制丝杆
};

struct MasterCmd
{
    MasterCmdType cmd_type;
};

struct MasterCmdMotion
{
    MasterCmdType cmd_type;
    int16_t x_speed;
    int16_t y_speed;
    int16_t z_speed;
};

struct MasterCmdServo
{
    MasterCmdType cmd_type;
    uint8_t id;
    uint16_t angle;
    uint16_t time;
};

struct MasterCmdScrew
{
    MasterCmdType cmd_type;
    uint16_t pos;  //0~2000
};


#endif



#ifdef __cplusplus
extern "C" {
#endif

    void RobotInit();
    void RobotTick();

    void RobotTest();
    void RobotMain();

    void Robot_DbusMove();
    void Robot_Dbus_s11_s21();
    void Robot_Dbus_s11_s22();
    void Robot_Dbus_s12();
    void Robot_Dbus_s13_s23();
    void Robot_Dbus_s13_s21();


    void RobotRecvMasterCmdThread();
//    void HandlePingRequest(MasterCmd* cmd);
//    void HandleMotionRequest(MasterCmd* cmd);
//    void HandleServeRequest(MasterCmd* cmd);
//    void HandleScrewRequest(MasterCmd* cmd);

    void OnHAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);
    void OnHAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
    void OnHAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
    void OnHAL_UART_ErrorCallback(UART_HandleTypeDef* huart);
    void OnHAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
    void OnHAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
    void RecvDR16Thread();


#ifdef __cplusplus
};
#endif
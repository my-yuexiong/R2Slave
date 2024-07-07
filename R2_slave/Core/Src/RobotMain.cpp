/*
 * @author BusyBox
 * @date 2024/4/27
 * @version 1.0
 */

#include "RobotMain.h"

std::shared_ptr<SerialPort> serial_port;
std::shared_ptr<SerialPort> master_port;
std::shared_ptr<ReliableBinaryTransfer> transfer;
std::shared_ptr<C6xxController> c6xx_controller1;
std::shared_ptr<M3508Motor> m3508_motor1;
std::shared_ptr<M3508Motor> m3508_motor2;
std::shared_ptr<M3508Motor> m3508_motor3;
std::shared_ptr<M3508Motor> m3508_motor4;
std::shared_ptr<OmnidirectionalMotion> motion;
std::shared_ptr<HuanerProtocol> ServoControl;
std::shared_ptr<Screw> MyScrew;

uint32_t motion_timeout = 0;
uint16_t screw_pos = 0;

void RobotInit()
{
    serial_port = std::make_shared<SerialPort>(&huart8);
    serial_port->start();
    master_port = std::make_shared<SerialPort>(&huart6);
    master_port->start();

    c6xx_controller1 = std::make_shared<C6xxController>(&hcan1);

    m3508_motor1 = std::make_shared<M3508Motor>(c6xx_controller1, 1, DjiMotor::Mode::SPEED);
    m3508_motor2 = std::make_shared<M3508Motor>(c6xx_controller1, 2, DjiMotor::Mode::SPEED);
    m3508_motor3 = std::make_shared<M3508Motor>(c6xx_controller1, 3, DjiMotor::Mode::SPEED);
    m3508_motor4 = std::make_shared<M3508Motor>(c6xx_controller1, 4, DjiMotor::Mode::SPEED);

    transfer = std::make_shared<ReliableBinaryTransfer>(master_port);
    ServoControl = std::make_shared<HuanerProtocol>(serial_port);

    MyScrew = std::make_shared<Screw>(&htim2, GPIOA, GPIO_PIN_4);

    m3508_motor1->set_target_rpm(0);
    m3508_motor2->set_target_rpm(0);
    m3508_motor3->set_target_rpm(0);
    m3508_motor4->set_target_rpm(0);

    motion = std::make_shared<OmnidirectionalMotion>(m3508_motor1, m3508_motor2, m3508_motor3, m3508_motor4);
    motion_timeout = HAL_GetTick();

}
//void RobotMain()
//{
//    while(true)
//    {
//        HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);
//        HAL_Delay(100);
//
//        if(dr16->alive() )
//        {
//
//            while(dr16->get_s2()==1)
//            {
//                while(dr16->get_s1()==1)
//                {
//                    Robot_Dbus_s11_s21();
//                }
//                while(dr16->get_s1()==3)
//                {
//                    Robot_Dbus_s11_s22();
//                }
//
//            }
//            while(dr16->get_s2()==3)
//            {
//                Robot_Dbus_s12();
//            }
//            while(dr16->get_s2()==2)
//            {
//                if(dr16->get_s1()==3)
//                {
//                    Robot_Dbus_s13_s23();
//                }
//                else if(dr16->get_s1()==1)
//                {
//                    Robot_Dbus_s13_s21();
//                }
//            }
//
//        }
//
//
//
//    }
//
//}
void RobotTest()
{
    while (true) {
        HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);
        HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
        HAL_Delay(100);
        ServoControl->move_time_write(1, 500, 400);
        HAL_Delay(450);
    }
}

void HandlePingRequest(MasterCmd* cmd)
{
    transfer->send_binary(reinterpret_cast<uint8_t*>(cmd), sizeof(MasterCmd));
}

void HandleMotionRequest(MasterCmd* cmd)
{
    MasterCmdMotion* motion_cmd = reinterpret_cast<MasterCmdMotion*>(cmd);
    motion->clear();
    motion->add_x_speed(motion_cmd->x_speed);
    motion->add_y_speed(motion_cmd->y_speed);
    motion->add_z_speed(motion_cmd->z_speed);
    motion->commit();
    motion_timeout = HAL_GetTick();
}

void HandleServeRequest(MasterCmd* cmd)
{
    HAL_GPIO_TogglePin(LEDG_GPIO_Port,LEDG_Pin);
    MasterCmdServo* serve_cmd = reinterpret_cast<MasterCmdServo*>(cmd);
    ServoControl->move_time_write(serve_cmd->id, serve_cmd->angle, serve_cmd->time);
}

void HandleScrewRequest(MasterCmd* cmd)
{
    MasterCmdScrew* screw_cmd = reinterpret_cast<MasterCmdScrew*>(cmd);
    MyScrew->writePos(screw_cmd->pos);
}

void RobotTick()
{
    c6xx_controller1->tick();
//    c6xx_controller2->tick();
    motion->tick();

    if (abs(HAL_GetTick() - motion_timeout) > 1000) {
        motion->clear();
        motion->commit();
    }
}

void RobotRecvMasterCmdThread()
{
    while (true) {
        uint8_t recv_buff[256];
        std::size_t recv_len = transfer->recv_binary(recv_buff, 256, 200);
//        transfer->send_binary(recv_buff, 128);
//        master_port->write(recv_buff, 256);

        HAL_GPIO_TogglePin(LEDR_GPIO_Port, LEDR_Pin);
        if (recv_len == 0) continue;
//        HAL_GPIO_TogglePin(LEDR_GPIO_Port, LEDR_Pin);

        MasterCmd* ret_cmd = reinterpret_cast<MasterCmd*>(recv_buff);
        switch (ret_cmd->cmd_type) {
            case MasterCmdType::Ping:
                HandlePingRequest(ret_cmd);
                break;
            case MasterCmdType::Motion:
                HandleMotionRequest(ret_cmd);
                break;
            case MasterCmdType::Servo:
                HandleServeRequest(ret_cmd);
                break;
            case MasterCmdType::Screw:
                HandleScrewRequest(ret_cmd);
                break;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    serial_port->OnHAL_UARTEx_RxEventCallback(huart, size);
    master_port->OnHAL_UARTEx_RxEventCallback(huart, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_TxCpltCallback(huart);
    master_port->OnHAL_UART_TxCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_RxCpltCallback(huart);
    master_port->OnHAL_UART_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_ErrorCallback(huart);
    master_port->OnHAL_UART_ErrorCallback(huart);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c6xx_controller1->OnHAL_CAN_RxFifo0MsgPendingCallback(hcan);
//    c6xx_controller2->OnHAL_CAN_RxFifo0MsgPendingCallback(hcan);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c6xx_controller1->OnHAL_CAN_RxFifo1MsgPendingCallback(hcan);
//    c6xx_controller2->OnHAL_CAN_RxFifo1MsgPendingCallback(hcan);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        MyScrew->OnHAL_TIM_PWM_PulseFinishedCallback();
    }
}
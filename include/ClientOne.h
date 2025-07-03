#ifndef CLIENTONE_H
#define CLIENTONE_H


/******************************************************************************
 * @file        ClientOne.h
 * @brief       简要说明
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2022-04-22
 * @version     v1.3
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2021-11-21 Version: v1.0
 * @par 描述： 稳定版本。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-17 Version: v1.1
 * @par 描述：添加修改广播频率的接口。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-21 Version: v1.2
 * @par 描述：添加用户注册的可访问内存地址。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-22 Version: v1.3
 * @par 描述：获取机器人状态。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-06-10 Version: v1.4
 * @par 描述：增加对广播数据的合并解析。
 * </table>
 *****************************************************************************/


#include "TCPClientBase.h"
#include "ClientOneProtocol.h"
#include <string>
#include <vector>

namespace AeroRobot {

class ClientOne : public TCPClientBase
{
public:
    ClientOne(const char * IP, int port = 13000, bool newDebugPrint = false, bool clientBlock = false);

    //Virtual to process protocol if received.
    bool protocolProcess() override;

    //Get robotStatus.
    const RobotStatus & getRobotStatus();
    void writeRobotStatus(std::ostream & os);

    //Control CMD.
    void setEmergencyOn();
    void setEmergencyOff();
    void setServoOn();
    void setServoOff();
    void moveStop();
    void MoveContinuousJoint(int axis, bool forwardMove);                           //axis: [1, 6]. forwardMove: true,false -> forward, backward.
    void MoveContinuousCartesianPose(int dir, bool forwardMove, int refCoordinate); //dir: [1, 3] -> x,y,z. forwardMove: true,false -> forward, backward. refCoordinate: 0, 1 -> base, TCP
    void MoveContinuousCartesianRota(int dir, bool forwardMove, int refCoordinate); //dir: [1, 3] -> rx,ry,rz. forwardMove: true,false -> forward, backward. refCoordinate: 0, 1 -> base, TCP

    void moveInchingPos(int refCoordinate, int dir, bool forwardMove); //refCoordinate: 0, 1 -> base, TCP. dir: [0, 2] -> x,y,z. forwardMove: true,false -> forward, backward.
    void moveInchingRot(int refCoordinate, int dir, bool forwardMove); // refCoordinate: 0, 1 -> base, TCP. dir: [0, 2] -> rx,ry,rz. forwardMove: true,false -> forward, backward.

    void JogSpeedUp();
    void JogSpeedDown();

    /**
     * @brief changeBroadcastCycTime 改变广播周期。默认是100ms。
     * @param time 单位是ms,支持修改最快8ms。
     */
    void changeBroadcastCycTime(int time);


    /**
     * @brief getRobotStatusPtrRegister 用户自定义RobotStatus类型的数据结构然后将数据指针通过本函数注册一次，自定义的数据结构便会自动更新。
     * @param userData_ptr 用户传入的RobotStatus数据指针。
     * @attention ！！！用户需要确保在程序运行过程中，指针指向的数据内存始终是合法的、可访问的。！！！
     * @return 注册成功返回true。
     */
    bool getRobotStatusPtrRegister(RobotStatus * userData_ptr);


    /**
     * @brief getRobotState 获取机器人的状态。
     * @param robotState 传入被更新的数据引用。
     */
    void getRobotState(RobotState & robotState);


private:
    bool dataDecode();
    bool businessProcess();

    bool checkAxisNum(int axis); //[1, 6]
    void stringToDouble6(const std::vector<std::string>::const_iterator &stringDataIter, double * data);

private:

    /**
     * @brief robotStatus 数据更新内存。
     */
    RobotStatus robotStatus;


    /**
     * @brief user_robotState_Ptr 存储用户提供的可访问数据内存地址。
     */
    RobotStatus * user_robotState_Ptr;


    /**
     * @brief robotState 记录机器人状态。
     */
    RobotState robotState;


};

}

#endif // CLIENTONE_H

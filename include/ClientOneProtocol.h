#ifndef CLIENTONEPROTOCOL_H
#define CLIENTONEPROTOCOL_H

/******************************************************************************
 * @file        ClientOneProtocol.h
 * @brief       简要说明
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2022-04-26
 * @version     v1.2
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2021-11-21 Version: v1.0
 * @par 描述：稳定版。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-21 Version: v1.1
 * @par 描述：增加RobotState结构。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-26 Version: v1.2
 * @par 描述：增加RobotStatus类型的描述注释。
 * </table>
 *****************************************************************************/



#include <iostream>

namespace AeroRobot {

/**
 * @brief The RobotState struct 机器人
 */
struct RobotState
{
    /**
     * @brief isEmergency true:处在急停状态；false:不在急停状态。
     */
    bool isEmergency;


    /**
     * @brief isServoOn true:处在上伺服状态；false:处在下伺服状态。
     */
    bool isServoOn;


    /**
     * @brief errorJoints 从1开始计数，返回出错的关节。比如“123”代表1、2、3关节出错。
     */
    int errorJoints;


    /**
     * @brief isRunning true:机器人正在运动；false:机器人没有运动。
     */
    bool isRunning;

};


/**
 * @brief The RobotStatus struct 机器人状态数据类型。
 */
struct RobotStatus
{
    /**
     * @brief name 协议名字。目前是"iPad"。名字固定。
     */
    char name[10];


    /**
     * @brief axisNum 轴的数量。6轴机械臂返回6。
     */
    int axisNum;


    /**
     * @brief currentJoint 机械臂当前关节角度。单位是deg。
     */
    double currentJoint[6];


    /**
     * @brief currentPoseEuler 机械臂的当前笛卡尔空间位姿。单位分别是mm和deg。
     */
    double currentPoseEuler[6];


    /**
     * @brief currentPoseEulerTCP 机械臂当前笛卡尔空间TCP的位姿。单位分别是mm和deg。
     */
    double currentPoseEulerTCP[6];


    /**
     * @brief JogSpeedJoint JOG运动时关节空间运动速度。单位是deg/s。
     */
    double JogSpeedJoint;


    /**
     * @brief JogSpeedPose JOG运动时笛卡尔空间的位置速度。单位是mm/s。
     */
    double JogSpeedPose;


    /**
     * @brief JogSpeedEuler JOG运动时笛卡尔空间的旋转速度。单位是deg/s。
     */
    double JogSpeedEuler;


    /**
     * @brief JogInchingPose JOG运动时笛卡尔空间直线运动的Inching距离。单位是mm。
     */
    double JogInchingPose;


    /**
     * @brief JogInchingEuler JOG运动时笛卡尔空间旋转运动的Inching角度。单位是deg。
     */
    double JogInchingEuler;


    /**
     * @brief currentSpeedJoint 机械臂当前关节速度。单位是rad/s。
     */
    double currentSpeedJoint[6];


    /**
     * @brief currentAmpereJoint 机械臂当前电流。单位是A。
     */
    double currentAmpereJoint[6];
};

std::ostream & operator<<(std::ostream & os, RobotStatus & data);

}



#endif // CLIENTONEPROTOCOL_H

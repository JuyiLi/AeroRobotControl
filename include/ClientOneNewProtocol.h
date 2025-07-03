#ifndef CLIENTONENEWPROTOCOL_H
#define CLIENTONENEWPROTOCOL_H

/******************************************************************************
 * @file        ServerOneNewProtocol.h
 * @brief       新1服务器的协议
 * @details     详细说明
 * @author      ARC<1131970238@qq.com>
 * @date        2023-03-16
 * @version     v1.1
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2023-02-16 Version: v1.0
 * @par 描述： 从ServerThreeProtocol拷贝。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-03-16 Version: v1.1
 * @par 描述： 加入了JOGSPEED的获取
 * </table>
 *****************************************************************************/


#include <string>
#include <ostream>
#include "spdlog/fmt/ostr.h"


namespace AeroRobot {

/**
 * @brief The RobotStatus struct 机器人状态数据类型。
 */
struct RobotStatus
{
    std::string pVersion;//@brief 协议的版本。
    long broadTimes; //@brief 广播的次数/本次数据的编号。
    std::string armType;//@brief 机械臂类型。
    double JointCurPos[6];//@brief currentJoint 机械臂当前关节角度。单位是rad。
    double JointCurVel[6];//@brief currentSpeedJoint 机械臂当前关节速度。单位是rad/s。
    double JointCurAcc[6];//@brief currentAmpereJoint 机械臂当前电流。单位是rad/s/s。
    double JointCurAmp[6];//@brief currentAmpereJoint 机械臂当前电流。单位是A。
    double CartTCPPosEuler[6];//@brief currentPoseEuler 机械臂的当前笛卡尔空间位姿。单位分别是m和rad。
    int JogSpeedLevel; //@brief Jog速度档位。
    double JogSpeed[3]; //@brief Jog速度:rad/s, m/s, rad/s
    int emergencyState; //1代表开启急停；0代表关闭急停。
    int servoState;//1代表上伺服；0代表下伺服。
    int runningState;//1代表正在运动；0代表停止运动。
};

inline void writeDouble6(std::ostream & os, const std::string & pre, const double * data)
{
    os << pre;
    for(size_t i = 0; i < 6; ++i){
        os << " " << data[i];
    }
}

inline std::ostream & operator<<(std::ostream & os, const RobotStatus & data)
{
    os << " pVersion: " << data.pVersion << " broadTimes: " << data.broadTimes << " armType: " << data.armType
       << " emergencyState: " << data.emergencyState << " servoState: " << data.servoState << " runningState: " << data.runningState << std::endl;
    writeDouble6(os, " CurJPos: ", data.JointCurPos); os << std::endl;
    writeDouble6(os, " CurJVel: ", data.JointCurVel); os << std::endl;
    writeDouble6(os, " CurJAcc: ", data.JointCurAcc); os << std::endl;
    writeDouble6(os, " CurJAmp: ", data.JointCurAmp); os << std::endl;
    writeDouble6(os, " CurPoseEuler: ", data.CartTCPPosEuler); os << std::endl;
    os << " Jog Speed: [" << data.JogSpeedLevel << "] " << data.JogSpeed[0] << " " << data.JogSpeed[1] << " " << data.JogSpeed[2];

    return os;
}



}


#endif // CLIENTONENEWPROTOCOL_H

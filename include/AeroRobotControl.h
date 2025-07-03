#ifndef AEROROBOTCONTROL_H
#define AEROROBOTCONTROL_H

/******************************************************************************
 * @file        AeroRobotControl.h
 * @brief       为用户提供二次开发的对象库。
 * @details     包含一个类AeroRobotControl。定义一个对象以代表要进行控制的机械臂。
 * @author      ARC<1131970238@qq.com>
 * @date        2023-09-07
 * @version     v1.19
 * @copyright   Copyright By ARC, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2022-03-15 Version: v1.0
 * @par 描述：添加新的注释信息；添加远程编程的函数接口。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-16 Version: v1.1
 * @par 描述：添加获取双编码差值的接口，还未实现。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-04-17 Version: v1.2
 * @par 描述：实现双编码差值的接口；添加修改广播频率的接口。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-17 Version: v1.3
 * @par 描述： 支持快速编程。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-04-20 Version: v1.4
 * @par 描述： 添加基于TCP的增量点位运动。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-04-21 Version: v1.5
 * @par 描述： 添加以注册数据指针的方式获取数据。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-04-22 Version: v1.6
 * @par 描述：获取机器人状态。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-04-26 Version: v1.7
 * @par 描述：实现机器人inching获取。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-05-28 Version: v1.8
 * @par 描述：控制客户端是否打印错误。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-06-27 Version: v1.9
 * @par 描述：修改了双编码获取数值的类型，需要结合0627版本的控制器使用。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-09-21 Version: v1.10
 * @par 描述：添加了TCP操作相关函数。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-10-10 Version: v1.11
 * @par 描述：构造函数中加入默认参数RobotInfoAccess, 可以选择机器人信息的获取来源，是广播式还是应答式。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-11-04 Version: v1.12
 * @par 描述：回HOME和ZERO位置运动速度为rad/s。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-11-30 Version: v1.13
 * @par 描述：1.修复moveJointToUntilFinish和moveTaskToUntilFinish在遇到机械臂紧急停止情况下返回true的bug。
 *           2.修改了JOG的注释。
 *           3.加入数字日期版本。
 *           4.修改获取信息方式的enum名字。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-02-18 Version: v1.14
 * @par 描述：替换了新的客户端1和客户端3.去掉部分topic函数：inching、注册数据指针；加入获取紧急停止、伺服、运动状态的函数。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-03-02 Version: v1.15
 * @par 描述：加入六位力传感器数据获取。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-03-16 Version: v1.16
 * @par 描述： 加入了JOGSPEED的获取
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-04-12 Version: v1.17
 * @par 描述： 加入了获取目标角度和位姿的函数。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-05-23 Version: v1.18
 * @par 描述： 加入了获取emergency,servo,running状态的新服务器3的版本。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2023-09-07 Version: v1.19
 * @par 描述： 加入了获取前置伺服轴的协议
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2024-03-26 Version: v1.20
 * @par 描述： getCurrentPoseJointSpace和笛卡尔函数返回值变为bool.
 * </table>
 *****************************************************************************/



#include <memory>
//#include "ClientOne.h"
//#include "ClientThree.h"
#include "ClientOneNew.h"
#include "ClientThreeNew.h"



namespace AeroRobot {

enum{
    INFO_BROADCAST_WAY,
    INFO_SERVICE_WAY
};

/**
 * @brief The AeroRobotControl class 机械臂运动控制类。
 */
class AeroRobotControl
{
public:

    //======================================================================== 初始化函数 =====================================================================

    /**
     * @brief AeroRobotControl 构造函数初始化时需要输入机械臂的IP。
     * @param IP
     */
    AeroRobotControl(const char * IP, int robotInfoAcess = INFO_BROADCAST_WAY);

    ~AeroRobotControl();


    /**
     * @brief connectToRobot 连接机械臂。
     * @return 连接成功如果返回true,否则返回false。
     */
    bool connectToRobot();

    /**
     * @brief disconnectToRobot 断开与机械臂的连接。
     */
    void disconnectToRobot();

    //=======================================================================================================================================================



    //====================================================================== 机械臂通用控制 ====================================================================

    /**
     * @brief setEmergencyOn 打开机器人的急停。机器人会停止当前的一切运动并不在响应后续的运动。
     */
    void setEmergencyOn();


    /**
     * @brief setEmergencyOff 关闭机器人急停。机器人可以继续响应运动命令。
     */
    void setEmergencyOff();


    /**
     * @brief setServoOn 为机器人上伺服。
     */
    void setServoOn();


    /**
     * @brief setServoOff 为机器人下伺服。
     */
    void setServoOff();


    /**
     * @brief setHomePos 设置当前位置为Home位置.
     * @return 成功如果返回true,否则返回false。
     */
    bool setHomePos();

    /**
     * @brief moveToHomePos 运动到Home位置.
     * @param s 单位是rad/s。
     * @return 成功如果返回true,否则返回false。
     */
    bool moveToHomePos(double s);

    /**
     * @brief moveToZeroPos 运动到Zero位置.
     * @param s 单位是rad/s。
     * @return 成功如果返回true,否则返回false。
     */
    bool moveToZeroPos(double s);



    /**
     * @brief changeBroadcastCycTime 改变广播周期。
     * @param time 单位是ms,支持修改最快8ms。
     */
    //void changeBroadcastCycTime(int time);



    //=======================================================================================================================================================


    //====================================================================== 机械臂状态获取 ====================================================================
    /**
     * @brief getCurrentPoseJointSpace 获取机器人在关节空间的当前位置。
     * @param pos_ptr 需要用户提前定义一个double 6 的数组，并将数组的地址作为输入参数。数据会被更新到用户数组中。单位是rad。
     * @return false说明请求的数据无效，不能使用。(可能包括请求数据超时，数据收到但解析错误：数据协议、帧数等)(INFO_BROADCAST_WAY总是true)
     */
    bool getCurrentPoseJointSpace(double * pos_ptr);



    /**
     * @brief getCurrentPoseCartSpace 获取机器人在笛卡尔空间的当前位置和姿态。
     * @param pos_ptr 需要用户提前定义一个double 6 的数组，并将数组的地址作为输入参数。数据会被更新到用户数组中。
     *                前三个数值分别是x,y,z，单位是m；后三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。
     * @return false说明请求的数据无效，不能使用。(可能包括请求数据超时，数据收到但解析错误：数据协议、帧数等)(INFO_BROADCAST_WAY总是true)
     */
    bool getCurrentPoseCartSpace(double * pos_ptr);



    /**
     * @brief getCurrentSpeedJointSpace  获取机器人在关节空间的当前速度。
     * @param speed_ptr 需要用户提前定义一个double 6 的数组，并将数组的地址作为输入参数。数据会被更新到用户数组中。单位是rad/s。
     */
    void getCurrentSpeedJointSpace(double * speed_ptr);

    /**
     * @brief getCurrentAccelJointSpace
     * @param accel_ptr
     */
    void getCurrentAccelJointSpace(double * accel_ptr);


    /**
     * @brief getCurrentAmpereJointSpace  获取机器人在关节空间的当前电流。
     * @param Ampere_ptr 需要用户提前定义一个double 6 的数组，并将数组的地址作为输入参数。数据会被更新到用户数组中。单位是A。
     */
    void getCurrentAmpereJointSpace(double * Ampere_ptr);

    /**
     * @brief getCurrentJointTemperature 获取当前机器人6个关节的温度
     * @param temper 需要用户提前定义一个double 6 的数组，并将数组的地址作为输入参数。数据会被更新到用户数组中。单位是摄氏度。
     */
    void getCurrentJointTemperature(double * temper);


    /**
     * @brief getDoubleEncoderDiff 获取机器人双编码器的插值。目前只有泰科RDM驱动器可以获取有效数据。不受广播频率的影响。
     * @param diff_ptr 需要用户提前定义一个double 6 的数组，并将数组的地址作为输入参数。数据会被更新到用户数组中。单位是rad。
     * @return 如果没有超过请求时间并且通讯正常返回true。
     * @todo 还未测试。
     */
    bool getDoubleEncoderDiff(double * diff_ptr);


    /**
     * @brief isHome 查询是否在Home位置。
     * @return 通信正常且在Home位置返回true。
     */
    bool isHome();


    /**
     * @brief isEmergencyOn
     * @return true代表急停打开；false代表急停关闭。
     */
    bool isEmergencyOn();


    /**
     * @brief isServoOn
     * @return true代表上伺服；false代表下伺服。
     */
    bool isServoOn();


    /**
     * @brief isRunning
     * @return true代表正在运动；false代表停止运动。
     */
    bool isRunning();


    /**
     * @brief getRobotStatusPtrRegister 用户注册RobotStatus类型的数据结构，并将数据指针传入，库函数会自动更新数据。
     * @param userData_ptr 用户传入的数据指针。
     * @attention ！！！用户需要确保在程序运行过程中，指针指向的数据内存始终是合法的、可访问的。！！！
     * @attention ！！！获取数据的单位与直接通过"get***"函数获取数据的单位可能会不一致。请用户仔细查看"RobotStatus"的注释。！！！
     * @return 注册成功返回true。
     */
    //bool getRobotStatusPtrRegister(RobotStatus * userData_ptr);


    /**
     * @brief getRobotState 获取机器人的状态。
     * @param robotState 传入被更新的数据引用。
     */
    //void getRobotState(RobotState & robotState);

    //========================================================================================================================================================


    //======================================================================= 机械臂TCP设置 ====================================================================

    /**
     * @brief getCurrentRobotTCP 获取当前Robot TCP。
     * @param TCPName 返回当前TCP的name。
     * @param tcpPosEuler 返回当前TCP的位置和欧拉角。前三个数值分别是x,y,z，单位是m；后三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。
     * @return 获取正确返回true。
     */
    bool getCurrentRobotTCP(std::string & TCPName, std::array<double, 6> & tcpPosEuler);


    /**
     * @brief setCurrentRobotTCP 设置name为"TCPName"的Robot TCP为被选中的当前TCP。
     * @param TCPName Robot TCP的name。
     * @return 如果控制端没有name为"TCPName"的TCP信息，则会返回false。
     */
    bool setCurrentRobotTCP(const std::string & TCPName);


    /**
     * @brief setNewCurrentRobotTCP 输入一个新的Robot TCP并设置为当前被选中的Robot TCP。如果控制段已经存在同名的TCP，则会将其覆盖。
     * @param TCPName TCP的name。
     * @param tcpPosEuler TCP的位置和欧拉角。前三个数值分别是x,y,z，单位是m；后三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。
     * @return
     */
    bool setNewCurrentRobotTCP(const std::string & TCPName, const std::array<double, 6> & tcpPosEuler);





    //========================================================================================================================================================


    //===================================================================== 机械臂JOG示教控制 ==================================================================
    /**
     * @brief MoveContinuousJoint 关节空间机械臂单方向JOG运动。
     * @param axis 运动的关节轴：范围：[1~6]
     * @param forwardMove true:正转；false：反转。
     */
    bool MoveContinuousJoint(int axis, bool forwardMove);


    /**
     * @brief MoveContinuousCartesianPose 笛卡尔空间机械臂单方向位置JOG运动。
     * @param dir 运动的方向。范围：[1, 2, 3]代表[x, y, z]方向的运动。
     * @param forwardMove true:正转；false：反转。
     * @param refCoordinate 运动参考坐标系：范围：[0, 1]代表基于[base, TCP]坐标系运动。
     */
    bool MoveContinuousCartesianPose(int dir, bool forwardMove, int refCoordinate);


    /**
     * @brief MoveContinuousCartesianRota 笛卡尔空间机械臂单方向旋转JOG运动。
     * @param dir 运动的方向。范围：[1, 2, 3]代表[rx, ry, rz]方向的运动。
     * @param forwardMove true:正转；false：反转。
     * @param refCoordinate 运动参考坐标系：范围：[0, 1]代表基于[base, TCP]坐标系运动。
     */
    bool MoveContinuousCartesianRota(int dir, bool forwardMove, int refCoordinate);

    /**
     * @brief getJogSpeedLevel
     * @return
     */
    int getJogSpeedLevel();

    /**
     * @brief getJogSpeedVal
     * @param data rad/s, m/s, rad/s
     */
    void getJogSpeedVal(double data[3]);


    /**
     * @brief JogSpeedUp JOG加速一档。
     */
    void JogSpeedUp();


    /**
     * @brief JogSpeedDown JOG减速一档
     */
    void JogSpeedDown();


    /**
     * @brief 设置JOG运行速度档位：1～10。
     */
    bool JogSpeedDirectSet(int val);


    /**
     * @brief JogMoveStop 停止JOG运动。
     */
    void JogMoveStop();


    //========================================================================================================================================================

    //======================================================================= 机械臂点位运动 ====================================================================
    /**
     * @brief set_csp 设置机械臂处于CSP运动模式。
     * @return 通讯成功并且设置成功返回true。
     */
    bool set_csp();


    /**
     * @brief isMoveFinished 查询机械臂点位运动是否运动停止。这个函数只在CSP模式下有效。
     * @return 通许正确并且运动到位置会返回true。没有运动到位置会返回false。
     */
    bool isMoveFinished();


    /**
     * @brief set_joint_move_speed 设置机器人点位运动在关节空间的速度。这个函数只会影响CSP模式下的运动。
     * @param s 单位是rad/s。
     * @return 通讯正常并且设置成功会返回true。
     */
    bool set_joint_move_speed(double s);


    /**
     * @brief set_Task_move_speed 设置机器人点位运动在笛卡尔空间的移动/旋转速度。这个函数只会影响CSP模式下的运动。
     * @param s 是一个指向double2数组的指针，分别是移动速度和转动速度，单位分别是m/s和rad/s。
     * @return 通讯正常并且设置成功会返回true。
     */
    bool set_Task_move_speed(const double * s);


    /**
     * @brief moveJointTo 关节空间机器人点位运动。这个函数只在CSP模式下有效。
     * @param tQ 是一个指向double6数组的指针，表示期望机器人到达的绝对位置。单位是rad。
     * @return 通讯正常并且运动状态正常会返回true。
     */
    bool moveJointTo(double * tQ);

    /**
     * @brief moveJointToWithDefinedSpeed 关节空间机器人点位运动（以速度为基准）。这个函数只在CSP模式下有效。
     * @param tQ 是一个指向double7数组的指针，前6个数据表示期望机器人到达的绝对位置。单位是rad。最后一个数据表示机器人运动速度。单位是rad/s。
     * @return 通讯正常并且运动状态正常会返回true。
     */
    bool moveJointToWithDefinedSpeed(double * tQ);

    /**
     * @brief test 关节空间机器人速度运动测试。这个函数只在CSP模式下有效。
     * @param tS 是一个指向double7数组的指针，表示期望机器人到达的绝对速度。单位是rad/s。
     * @return 通讯正常并且运动状态正常会返回true。
     */
    bool test(double * tS);


    /**
     * @brief moveJointToUntilFinish 关节空间机器人点位运动。这个函数只在CSP模式下有效。此函数是阻塞函数，直到运动完或者截止时间到才会返回。
     * @param tQ 是一个指向double6数组的指针，表示期望机器人到达的绝对位置。单位是rad。
     * @param deadline 截至时间。如果在截至时间内没有运动完会主动发送停止指令并返回。单位是s。
     * @return 通讯正确、规划正确并且在截至时间内运动完会返回true。
     */
    bool moveJointToUntilFinish(double * tQ, double deadline = 60);


    /**
     * @brief moveTaskTo 笛卡尔空间相对基坐标系的绝对直线运动。这个函数只在CSP模式下有效。
     * @param tC 指向一个double[6]数组的指针，表示期望机械臂运动到的绝对位姿。
     *           前三个数值分别是x,y,z，单位是m；后三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。
     * @return 通讯正确并且规划正确会立即返回true。
     */
    bool moveTaskTo(double * tC);


    /**
     * @brief moveTaskBy 笛卡尔空间相对基坐标系的增量直线运动。这个函数只在CSP模式下有效。
     * @param tC 指向一个double[6]数组的指针，表示期望机械臂运动到的增量位姿。
     *           前三个数值分别是x,y,z，单位是m；后三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。
     * @return 通讯正确并且规划正确会立即返回true。
     */
    bool moveTaskBy(double * tC);


    /**
     * @brief moveTaskBy 笛卡尔空间相对TCP坐标系的增量直线运动。这个函数只在CSP模式下有效。
     * @param tC 指向一个double[6]数组的指针，表示期望机械臂运动到的增量位姿。
     *           前三个数值分别是x,y,z，单位是m；后三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。
     * @return 通讯正确并且规划正确会立即返回true。
     */
    bool moveTaskByTCP(double * tC);


    /**
     * @brief moveTaskToWithSpeed 笛卡尔空间以给定速度相对基坐标系的绝对直线运动。这个函数只在CSP模式下有效。
     * @param tC 指向一个double[8]数组的指针，表示期望机械臂运动到的绝对位姿和对应的移动/转动速度。因为指定了运动速度，所以不受函数“set_Task_move_speed”所指定的速度的影响。
     *           前三个数值分别是x,y,z，单位是m；中间三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。后两个数值是移动速度和转动速度，单位分别是m/s和rad/s。
     * @return 通讯正确并且规划正确会立即返回true。
     */
    bool moveTaskToWithSpeed(double * tC);

    /**
     * @brief moveTaskToWithDefinedSpeed 笛卡尔空间以给定速度相对基坐标系的绝对直线运动（以速度为基准）。这个函数只在CSP模式下有效。
     * @param tC 指向一个double[8]数组的指针，表示期望机械臂运动到的绝对位姿和对应的移动/转动速度。因为指定了运动速度，所以不受函数“set_Task_move_speed”所指定的速度的影响。
     *           前三个数值分别是x,y,z，单位是m；中间三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。后两个数值是移动速度和转动速度，单位分别是m/s和rad/s。
     * @return 通讯正确并且规划正确会立即返回true。
     */
    bool moveTaskToWithDefinedSpeed(double * tC);


    /**
     * @brief moveTaskTo 笛卡尔空间相对基坐标系的绝对直线运动。这个函数只在CSP模式下有效。此函数是阻塞函数，直到运动完或者截止时间到才会返回。
     * @param tC 指向一个double[6]数组的指针，表示期望机械臂运动到的绝对位姿。
     *           前三个数值分别是x,y,z，单位是m；后三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。
     * @param deadline 截至时间。如果在截至时间内没有运动完会主动发送停止指令并返回。单位是s。
     * @return 通讯正确、规划正确并且在截至时间内运动完会返回true。
     */
    bool moveTaskToUntilFinish(double * tC, double deadline = 60);


    /**
     * @brief pMoveStop 立即停止一切点位运动。
     * @return 通讯正确并且停止成功会返回true。
     */
    bool pMoveStop();

    //========================================================================================================================================================


    //======================================================================= 机械臂连续运动 ====================================================================
    /**
     * @brief moveJointToContinuesWithSpeed 关节空间连续位置运动。需要给定每次运动的速度。
     * @param tJ代表指向一个double[7]数组的指针。
     *        前6个参数代表期望关节角度。单位是rad。后1个参数代表转动的速度。单位是rad/s。
     * @return 成功返回true，失败返回false。
     */
    bool moveJointToContinuesWithSpeed(double * tJ);

    /**
     * @brief moveJointToContinuesWithDurat 关节空间连续位置运动。需要给定每次运动的时间。
     * @param tJ代表指向一个double[7]数组的指针。
     *        前6个参数代表期望关节角度。单位是rad。后1个参数代表转动的时间。单位是s。
     * @return 成功返回true，失败返回false。
     */
    bool moveJointToContinuesWithDurat(double * tJ);

    /**
     * @brief moveTaskToContinues 笛卡尔空间基于基坐标系的连续位置运动。
     * @param tC代表指向一个double[8]数组的指针。
     *         前三个数值分别是x,y,z，单位是m；中间三个数值是rz,ry,rx的欧拉角（也是求取顺序），单位是rad。后两个数值是移动速度和转动速度，单位分别是m/s和rad/s。
     * @return 成功返回true，失败返回false。
     */
    bool moveTaskToContinues(double * tC);

    //=======================================================================================================================================================

    //================================================================= 机械臂复杂WayPoints运动 ===============================================================

    /**
     * @brief complexWayPointsPushBack 将复杂点加入控制器。每次调用都会在末尾加入一个新的wayoint。
     * @param tW代表指向一个double[6]数组的指针。代表要运动经过的关节空间的路径点。单位是rad。
     * @return 加入成功返回true，否则返回false。
     * @todo 已实现，还未测试。
     */
    bool complexWayPointsPushBack(double * tW);

    /**
     * @brief complexWayPointsClear 清除所有已经加入的wayPoints.
     * @return 清除成功返回true，否则返回false。
     * @todo  已实现，还未测试。
     */
    bool complexWayPointsClear();

    /**
     * @brief complexWayPointsRun 运行已经加入的所有wayPoints。
     * @return 运行成功返回true，否则返回false。
     * @todo  已实现，还未测试。
     */
    bool complexWayPointsRun();

    /**
     * @brief complexWayPointsRunIsFinish 检查运行是否已经结束。
     * @param ret true代表已经完成运动。false代表未完成运动。
     * @return true代表返回数据有效。false代表返回数据无效，可能通讯存在问题。
     */
    bool complexWayPointsRunIsFinish(bool & ret);

    /**
     * @brief pushLineSerialPoints 压入直线过程点位，等待运行。
     * @param tW double[6]数组指针，分别是x,y,z,rz,ry,rx，单位是m和rad。
     * @param LineSpeed 移动速度，单位是m/s。
     * @return
     */
    bool pushLineSerialPoints(double * tW, double LineSpeed);

    /**
     * @brief moveTaskToLineSerial 让已经压入的点位进行连续的直线运动。运行结束后会自动清空已有点位。
     * @return
     */
    bool moveTaskToLineSerial();

    /**
     * @brief clearLineSerialPoints 清空现有点位。
     * @return
     */
    bool clearLineSerialPoints();

    /**
     * @brief moveTaskToLineSerialPlan 对已经压入的点位进行规划（必须是多点）。
     * @return
     */
    bool moveTaskToLineSerialPlan();

    /**
     * @brief moveTaskToLineSerialRun 直线运动。运行结束后会自动清空已有点位。
     * @return
     */
    bool moveTaskToLineSerialRun();

    //=======================================================================================================================================================

    //===================================================================== 机械臂编程运动 ====================================================================

    /**
     * @brief openProg 打开程序文件。
     * @param fileName 程序文件名。
     * @param line 打开的程序的行数。
     * @param error 1：没有这个文件。2：文件内容读取失败。3:通讯错误。 0：无错误。
     * @return 打开正确返回true,否则返回false。
     * @todo 还未测试。
     */
    bool openProg(const std::string fileName, int &line, int & error);


    /**
     * @brief runProg 开始运行程序。
     * @param error 1：没有加载程序。2：机械臂处于非运行状态不可运行。3：程序点位无法到达。4：通讯错误。0：无错误。
     * @return 运行成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool runProg(int & error);

    /**
     * @brief stopProg 立即停止程序运行。
     * @param error 1：通讯错误。0：无错误。
     * @return 停止成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool stopProg(int & error);


    /**
     * @brief saveProg 保存程序。
     * @param error 1：没有加载程序。2：通讯错误。0：无错误。
     * @return 保存成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool saveProg(int & error);


    /**
     * @brief saveExitProg 保存并退出程序。
     * @param error 1：没有加载程序。2：通讯错误。0：无错误。
     * @return 保存并退出成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool saveExitProg(int & error);


    /**
     * @brief exitProg 不保存直接退出程序。
     * @param error 1：没有加载程序。2：通讯错误。0：无错误。
     * @return 退出成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool exitProg(int & error);


    /**
     * @brief pushBackPoint 将机械臂当前位置加入程序最后。
     * @param w 传入的wayPoint数据。
     * @param error 1：没有打开的程序文件。2：通讯错误。0：无错误。
     * @return 加入成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool pushBackPoint(const ProgramWayPoint & w, int &error);


    /**
     * @brief insertPoint 将机械臂当前位置插入程序。
     * @param w 传入的wayPoint数据。
     * @param error 1：没有打开的程序文件。2：插入位置大于程序记录点的数量。3：通讯错误。0：无错误。
     * @return 加入成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool insertPoint(const ProgramWayPoint & w, int &error);


    /**
     * @brief erasePoint 删除制定位置的机器人编程点。
     * @param index 删除的位置。
     * @param error 1：没有打开的程序文件。2：删除位置大于程序记录点的数量。3：通讯错误。0：无错误。
     * @return 删除成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool erasePoint(int index, int &error);


    /**
     * @brief replacePoint 将机械臂当前位置替换制定位置的机器人编程点。
     * @param w 传入的wayPoint数据。
     * @param error 1：没有打开的程序文件。2：替换位置大于程序记录点的数量。3：通讯错误。0：无错误。
     * @return 替换成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool replacePoint(const ProgramWayPoint & w, int & error);


    /**
     * @brief clearAllPoints 删除所有的已经记录的编程点。
     * @param error 1：没有打开的程序文件。2：通讯错误。0：无错误。
     * @return 删除成功返回true,否则返回false。
     * @todo 还未测试。
     */
    bool clearAllPoints(int & error);



    //=======================================================================================================================================================



    //=================================================================== 机械臂连续速度运动 ===================================================================
    /**
     * @brief set_csv 设置机器人工作在CSV模式。
     * @return 通讯成功并且设置成功返回true。
     */
    bool set_csv();


    /**
     * @brief moveJointSpeedTo 机器人关节空间连续速度运动。本函数只工作在CSV模式下。
     * @param ts 指向double6数组的指针，表示每次的期望速度。单位是rad/s。
     * @return 通讯成功并且设置成功返回true。
     */
    bool moveJointSpeedTo(const double * ts);

    //=======================================================================================================================================================


    //===================================================================== 前置伺服轴控制 ====================================================================

    /**
     *
     * @param tq double7的数组，前六个是目标位置，后一个是运行速度。单位是rad/s。
     * @return
     */
    bool preJointsMoveTo(double * tq, int & err);

    /**
     * @brief 停止运动
     * @return
     */
    bool preJointsStop(int & err);
    /**
     * @brief 上下伺服
     * @return
     */
    bool preJointsServoOn(int & err);
    bool preJointsServoOff(int & err);

    /**
     * @brief 查询是否运动到位置。如果没有运动到位则返回false。
     * @return
     */
    bool preJointsIsMoveFinished(int & err);


    /**
     * @brief 获取当前位置,速度，电流。
     * @param pos_ptr， vel_ptr， torque_ptr： 指向double6数组的指针，返回当前位置。单位是rad，rad/s, 。
     * @return
     */
    bool preJointsGetPos(double * pos_ptr, int &err);
    bool preJointGetVel(double * vel_ptr, int &err);
    bool preJointsGetTorque(double * torque_ptr, int &err);

    //=======================================================================================================================================================


    //===================================================================== 后置伺服轴控制 ====================================================================

    /**
     *
     * @param tq double7的数组，前六个是目标位置，后一个是运行速度。单位是rad/s。
     * @return
     */
    bool afterJointsMoveTo(double * tq, int & err);

    /**
     * @brief 停止运动
     * @return
     */
    bool afterJointsStop(int & err);
    /**
     * @brief 上下伺服
     * @return
     */
    bool afterJointsServoOn(int & err);
    bool afterJointsServoOff(int & err);

    /**
     * @brief 查询是否运动到位置。如果没有运动到位则返回false。
     * @return
     */
    bool afterJointsIsMoveFinished(int & err);


    /**
     * @brief 获取当前位置,速度，电流。
     * @param pos_ptr， vel_ptr， torque_ptr： 指向double6数组的指针，返回当前位置。单位是rad，rad/s, 。
     * @return
     */
    bool afterJointsGetPos(double * pos_ptr, int &err);
    bool afterJointGetVel(double * vel_ptr, int &err);
    bool afterJointsGetTorque(double * torque_ptr, int &err);

    //=======================================================================================================================================================


    //===================================================================== 机械臂外设控制 ====================================================================

    bool connectToSixDForce();
    bool disConnectToSixDForce();
    bool getSixDForceData(double * data_ptr);

    //=======================================================================================================================================================


    //===================================================================== 机械臂工具函数 ====================================================================


    /**
     *
     * @param tarJoints 输入一个想要进行转化的目标角度。单位是rad。
     * @param cartGot 输出计算得到的目标位姿。顺序：x, y, z, rz, ry, rx； 单位：m, rad
     * @return true代表数据可用；false代表数据不可用。如果计算结果没有问题则err=0.
     */
    bool getCartPosFromJoints(double * tarJoints, double * cartGot, int & err);

    /**
     *
     * @param tarCart 输入一个想要进行转化的目标位姿。顺序：x, y, z, rz, ry, rx； 单位：m, rad
     * @param jointsGot 输出计算得到的目标角度。单位是rad。
     * @return true代表数据可用；false代表数据不可用。如果逆解正确则err=0，否则不等于0，需要进行判断.
     */
    bool getJointsPosFromCartPos(double * tarCart, double * jointsGot, int & err);

    //=======================================================================================================================================================

    /**
     * @brief isPrintingError 控制是否打印错误信息。
     * @param p true: 打印错误信息；false: 不打印错误信息。
     */
    void isPrintingError(bool p);

    /**
     * @brief isBroadcastPrint 是否打开广播数据的打印。
     * @param p
     */
    void isBroadcastPrint(bool p);

private:

    std::string serverIP;
    //std::shared_ptr<ClientOne> clientTopic_ptr;
    //std::shared_ptr<ClientThree> clientService_ptr;
    std::shared_ptr<ClientOneNew> clientTopicNew_ptr;
    std::shared_ptr<ClientThreeNew> clientServiceNew_ptr;
    int mRobotInfoAccess;
    double version;

};

}

#endif // AEROROBOTCONTROL_H

#ifndef CLIENTTHREE_H
#define CLIENTTHREE_H

/******************************************************************************
 * @file        ClientThree.h
 * @brief       简要说明
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2022-04-20
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
 * @par 描述： 添加双编码差值的获取函数。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-17 Version: v1.2
 * @par 描述： 支持快速编程。busines函数中针对有关获取数据的命令将得到的数据进行赋值不同的变量，从而不会被别的新的数据所覆盖。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-04-20 Version: v1.3
 * @par 描述： 添加基于TCP的增量点位运动。
 * </table>
 *****************************************************************************/



#include "ClientThreeProtocol.h"
#include "TCPClientBase.h"
#include <array>
#include <iostream>

namespace AeroRobot {

class ClientThree : public TCPClientBase
{
public:
    ClientThree(const char * IP, int port = 8008, bool newDebugPrint = false, bool clientBlock = false);

    //Virtual to process protocol if received.
    bool protocolProcess() override;

    //Emergency.
    bool setEmergenyOn();
    bool setEmergencyOff();

    //Set servo state.
    bool setServeOn();
    bool setServeOff();
    bool moveToHome(double s);
    bool moveToZero(double s);
    bool setNewHomePos();

    //Get robotArm status.
    bool getCurrentJoints(double * joints);
    bool getCurrentCart(double * pose);
    bool getCurrentJointsSpeed(double * speed);
    bool getCurrentAmpere(double * Ampere);
    bool getCurrentJointAcce(double * acce);
    bool getCurrentJointTemperature(double *temper);
    bool isHome();

    /**
     * @brief getDoubleEncoderDiff 获取机器人双编码器的插值。目前只有泰科RDM驱动器可以获取有效数据。
     * @param diff_ptr 需要用户提前定义一个double 6 的数组，并将数组的地址作为输入参数。数据会被更新到用户数组中。单位是rad。
     * @return
     */
    bool getDoubleEncoderDiff(double * diff_ptr);


    //Set csp mode.
    bool set_csp();
    //Get pose control motion state.
    bool isMoveFinished();
    //Joint space pose control.
    bool set_joint_move_speed(double s);
    bool moveJointTo(double * tQ);
    //Cartesian space pose control.
    bool set_Task_move_speed(const double * s);
    bool moveTaskTo(double * tC);
    bool moveTaskBy(double * tC);
    bool moveTaskByTCP(double * tC);
    bool moveTaskToWithSpeed(double * tC);
    bool moveStop();

    //Set csv mode.
    bool set_csv();
    //Joint space speed control.
    bool moveJointSpeedTo(const double * ts);

    //Continues motion.
    bool moveTaskToContinues(double * tC);
    bool moveJointToContinueswithSpeed(double * tC);
    bool moveJointToContinueswithDurat(double * tC);

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

    //================================================================= 机械臂复杂WayPoints运动 ===============================================================

    /**
     * @brief complexWayPointsPushBack 将复杂点加入控制器。每次调用都会在末尾加入一个新的wayoint。
     * @param tW代表指向一个double[6]数组的指针。代表要运动经过的关节空间的路径点。单位是rad。
     * @return 加入成功返回true，否则返回false。
     * @todo 还未实现。
     */
    bool complexWayPointsPushBack(double * tW);

    /**
     * @brief complexWayPointsClear 清除所有已经加入的wayPoints.
     * @return 清除成功返回true，否则返回false。
     * @todo 还未实现。
     */
    bool complexWayPointsClear();

    /**
     * @brief complexWayPointsRun 运行已经加入的所有wayPoints。
     * @return 运行成功返回true，否则返回false。
     * @todo 还未实现。
     */
    bool complexWayPointsRun();

    //=======================================================================================================================================================

    //===================================================================== 机械臂编程运动 ====================================================================

    /**
     * @brief openProg 新建程序文件。
     * @param fileName 程序文件名。
     * @param error0：无错误。
     * @return 打开正确返回true,否则返回false。
     * @todo 还未实现。
     */
    bool newProg(const std::string fileName, int & error);

    /**
     * @brief openProg 打开程序文件。
     * @param fileName 程序文件名。
     * @param error 1：没有这个文件。2：文件内容读取失败。3:通讯错误。 0：无错误。
     * @return 打开正确返回true,否则返回false。
     * @todo 还未实现。
     */
    bool openProg(const std::string fileName, int & error);


    /**
     * @brief runProg 开始运行程序。
     * @param error 1：没有加载程序。2：机械臂处于非运行状态不可运行。3：程序点位无法到达。4：通讯错误。0：无错误。
     * @return 运行成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool runProg(int & error);

    /**
     * @brief stopProg 立即停止程序运行。
     * @param error 1：通讯错误。0：无错误。
     * @return 停止成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool stopProg(int & error);


    /**
     * @brief saveProg 保存程序。
     * @param error 1：没有加载程序。2：通讯错误。0：无错误。
     * @return 保存成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool saveProg(int & error);


    /**
     * @brief saveExitProg 保存并退出程序。
     * @param error 1：没有加载程序。2：通讯错误。0：无错误。
     * @return 保存并退出成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool saveExitProg(int & error);


    /**
     * @brief exitProg 不保存直接退出程序。
     * @param error 1：没有加载程序。2：通讯错误。0：无错误。
     * @return 退出成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool exitProg(int & error);


    /**
     * @brief pushBackPoint 将机械臂当前位置加入程序最后。
     * @param w 传入的wayPoint数据。
     * @param error 1：没有打开的程序文件。2：通讯错误。0：无错误。
     * @return 加入成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool pushBackPoint(const ProgramWayPoint & w, int &error);


    /**
     * @brief insertPoint 将机械臂当前位置插入程序。
     * @param w 传入的wayPoint数据。
     * @param index 插入的位置。
     * @param error 1：没有打开的程序文件。2：插入位置大于程序记录点的数量。3：通讯错误。0：无错误。
     * @return 加入成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool insertPoint(const ProgramWayPoint & w, int &error);


    /**
     * @brief erasePoint 删除制定位置的机器人编程点。
     * @param index 删除的位置。
     * @param error 1：没有打开的程序文件。2：删除位置大于程序记录点的数量。3：通讯错误。0：无错误。
     * @return 删除成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool erasePoint(int index, int &error);


    /**
     * @brief replacePoint 将机械臂当前位置替换制定位置的机器人编程点。
     * @param w 传入的wayPoint数据。
     * @param error 1：没有打开的程序文件。2：替换位置大于程序记录点的数量。3：通讯错误。0：无错误。
     * @return 替换成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool replacePoint(const ProgramWayPoint & w, int & error);


    /**
     * @brief clearAllPoints 删除所有的已经记录的编程点。
     * @param error 1：没有打开的程序文件。2：通讯错误。0：无错误。
     * @return 删除成功返回true,否则返回false。
     * @todo 还未实现。
     */
    bool clearAllPoints(int & error);


    /**
     * @brief updatePoints 更新程序的行数。
     * @param line 程序的行数。
     * @param error 0：无错误。
     * @return
     */
    bool updatePoints(int & line, int & error);

    //=======================================================================================================================================================


private:
    bool requestServer(RobotArmCMD cmdId, DataTypeEnum reqDataType, const void *requestData);
    void dataEncode(int cmdId, DataTypeEnum reqDataType, const void *reqData, char *sendBuff, int &sendbuffSize);
    bool dataDecode();
    bool businessProcess();
    bool dataDecodeAboutHeader(int &resDataSize);

    void resBoolWrite(std::ostream & os, const char * cmd, bool ret);
    void resDouble6Write(std::ostream & os, const char * cmd, double * double6);

    bool lockForServerRes;
    RobotArmCMD reqCMD;
    robotarmServiceData resData;
    bool resDataBool;
    int progLine;
    double doubleEncoderDiff[6];
    double currentJoints[6];
    double currentJointsSpeed[6];
    double currentJointAccel[6];
    double currentJointTemperature[6];
    double currentCart[6];
    double currentAmpere[6];

    /**
     * @brief currentRobotTCPInfo 存储从服务器请求获得的当前TCP信息。
     */
    RobotTCPInfo currentRobotTCPInfo;

};

}

#endif // CLIENTTHREE_H

#ifndef CLIENTTHREENEW_H
#define CLIENTTHREENEW_H

/******************************************************************************
 * @file        ClientThreeNew.h
 * @brief       基于evpp建立新的三号客户端。
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2023-03-08
 * @version     v1.2
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2023-02-05 Version: v1.0
 * @par 描述：
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-02-23 Version: v1.1
 * @par 描述：MSGReqRegister中加入错误玛的指针。实现错误玛的解析。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-03-08 Version: v1.2
 * @par 描述：更新协议版本为"Version1.1"。
 * </table>
 *****************************************************************************/

#include <string>
#include <thread>
#include <queue>
#include <set>
#include "ClientBaseEVPP.h"
#include "ClientThreeNewProtocol.h"

namespace AeroRobot {

class ClientThreeNew
{
public:

    struct MSGReqRegister{
        unsigned long msgIndex;
        RobotArmCMD reqCMD;
        bool * dataValid_ptr;
        bool isWait;
        robotarmServiceData * resData_ptr;
        int * errCode_ptr;
        bool *lockMsg_ptr;
    };

    /**
     * @brief ClientThreeNew 构造函数
     * @param serverAddr 形式："127.0.0.1:8008"
     * @param 客户端名字。
     */
    ClientThreeNew(const std::string& serverAddr, const std::string& name);

    /**
     * @brief connectToServer 连接服务器。
     * @return
     */
    bool connectToServer();

    /**
     * @brief Disconnect 断开服务器。
     */
    void Disconnect();

    /**
     * @brief isConnected
     * @return 建立建立返回true;否则返回false。
     */
    bool isConnected();

    /**
     * @brief isDebugPrint 控制是否打印本客户端的调试信息。
     * @param p true: 打印调试信息；false: 不打印调试信息。
     */
    void isDebugPrint(bool p){debugPrint = p;}

public:

    /**
     * @brief 急停控制
     * @return
     */
    bool setEmergenyOn();
    bool setEmergencyOff();

    /**
     * @brief setServeOn 伺服操作以及设置操作。
     * @return
     */
    bool setServeOn();
    bool setServeOff();
    bool moveToHome(double s);
    bool moveToZero(double s);
    bool setNewHomePos();
    bool isHome();

    /**
     * @brief getCurrentJoints 获取机械臂信息操作。
     * @return
     */
    bool getCurrentJoints(double * joints);
    bool getCurrentCart(double * pose);
    bool getCurrentJointsSpeed(double * speed);
    bool getCurrentAmpere(double * Ampere);
    bool getCurrentJointAcce(double * acce);
    bool getCurrentJointTemperature(double *temper);
    /**
     * @brief getDoubleEncoderDiff 获取机器人双编码器的插值。目前只有泰科RDM驱动器可以获取有效数据。
     * @param diff_ptr 需要用户提前定义一个double 6 的数组，并将数组的地址作为输入参数。数据会被更新到用户数组中。单位是rad。
     * @return
     */
    bool getDoubleEncoderDiff(double * diff_ptr);
    bool getRobotEmergencyState(bool * state_ptr, int & err);
    bool getRobotServoState(bool * state_ptr, int & err);
    bool getRobotRunningState(bool * state_ptr, int & err);
    bool getCartPosFromJoints(double * tarJoints, double * cartGot, int & err); //rad
    bool getJointsPosFromCartPos(double * tarCart, double * jointsGot, int & err); //x, y, z, rz, ry, rx; m, rad

    /**
     * @brief set_csp CSP下的点位运动。
     * @return
     */
    bool set_csp();
    //Get pose control motion state.
    bool isMoveFinished();
    //Joint space pose control.
    bool set_joint_move_speed(double s);
    bool moveJointTo(double * tQ);
    bool test(double * tS);
    bool moveJointBy(double * tQ);
    //Cartesian space pose control.
    bool set_Task_move_speed(const double * s);
    bool moveTaskTo(double * tC);
    bool moveTaskBy(double * tC);
    bool moveTaskByTCP(double * tC);
    bool moveTaskToWithSpeed(double * tC);
    bool moveStop();
    bool moveJointToWithDefinedSpeed(double * tQ);
    bool moveTaskToWithDefinedSpeed(double * tC);

    /**
     * @brief moveTaskToContinues CSP模式下可以随时改变轨迹的连续点位运动。
     * @return
     */
    bool moveTaskToContinues(double * tC);
    bool moveJointToContinueswithSpeed(double * tC);
    bool moveJointToContinueswithDurat(double * tC);

    bool JogMoveJointSpace(int axis, bool forwardMove); //axis: [0, 5]. forwardMove: true,false -> forward, backward.
    bool JogLineMoveCartesianSpace(int dir, bool forwardMove, int refCoordinate);//dir: [1, 3] ： x,y,z.； forwardMove: true,false : forward, backward.。refCoordinate: 0, 1 ： base, TCP
    bool JogRotMoveCartesianSpace(int dir, bool forwardMove, int refCoordinate);//dir: [1, 3] -> rx,ry,rz； forwardMove: true,false : forward, backward.。refCoordinate: 0, 1 ： base, TCP
    bool JogSpeedUp();
    bool JogSpeedDown();
    bool JogSpeedDirectSet(int val);//[1, 10]
    bool JogMoveStop();

    bool preJointsMoveTo(double * tq, int & err);
    bool preJointsStop(int & err);
    bool preJointsServoOn(int & err);
    bool preJointsServoOff(int & err);
    bool preJointsIsMoveFinished(int & err);
    bool preJointsGetPos(double * pos, int & err);
    bool preJointsGetVel(double * vel, int & err);
    bool preJointsGetTor(double * tor, int & err);

    bool afterJointsMoveTo(double * tq, int & err);
    bool afterJointsStop(int & err);
    bool afterJointsServoOn(int & err);
    bool afterJointsServoOff(int & err);
    bool afterJointsIsMoveFinished(int & err);
    bool afterJointsGetPos(double * pos, int & err);
    bool afterJointsGetVel(double * vel, int & err);
    bool afterJointsGetTor(double * tor, int & err);

    /**
     * @brief set_csv CSV模式下的速度运动。
     * @return
     */
    bool set_csv();
    //Joint space speed control.
    bool moveJointSpeedTo(const double * ts);



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

    /**
     * @brief complexWayPointsRunIsFinish 检查运行是否已经结束。
     * @param ret true代表已经完成运动。false代表未完成运动。
     * @return true代表返回数据有效。false代表返回数据无效，可能通讯存在问题。
     */
    bool complexWayPointsRunIsFinish(bool & ret);

    bool pushLineSerialPoints(double * tW, double LineSpeed);
    bool moveTaskToLineSerial();
    bool clearLineSerialPoints();
    bool moveTaskToLineSerialPlan();
    bool moveTaskToLineSerialRun();

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


    bool connectToSixDForce();
    bool disConnectToSixDForce();
    bool getSixDForceData(double * data_ptr);

    //=======================================================================================================================================================


private:
    /**
     * @brief requestServer
     * @param cmdId
     * @param reqDataType
     * @param requestData
     * @param resData_ptr
     * @return false说明请求的数据无效，不能使用。(可能包括请求数据超时，数据收到但解析错误：数据协议、帧数等)
     */
    bool requestServer(RobotArmCMD cmdId, DataTypeEnum reqDataType, const void *requestData, robotarmServiceData * resData_ptr, int * errCode);
    void dataEncode(int cmdId, uint32_t index, DataTypeEnum reqDataType, const void *reqData, char *sendBuff, int &sendbuffSize);
    MSGReqRegister & registerSendMsg(RobotArmCMD cmdId, DataTypeEnum reqDataType, const void *requestData, bool * dataValid_ptr, robotarmServiceData * resData, int * errCode_ptr, bool * msgLock);
    bool send(const std::string & msg);
    bool send(char * msg, int msgSize);
    //等待阻塞函数。如果lock在limit时间内没有变为false或者超过了limit时间则返回false;否则返回true。
    bool deadLine(const bool & lock, double limit = 1);


    /**
     * @brief CallBack接受数据进行处理的函数
     * @param message
     */
    void protocolProcess(const std::string& message);


    bool dataDecode(const std::string& message, MSGReqRegister & reqMsg);
    bool dataDecodeAboutHeader(const std::string& message, MSGReqRegister & reqMsg, robotarmServiceHeader &resHeader);


    void responsePrint(const MSGReqRegister & reqMsg);
    void resBoolPrint(const MSGReqRegister & reqMsg);
    void resIntArrayPrint(const MSGReqRegister & reqMsg, int num);
    void resDoubleArrayPrint(const MSGReqRegister & reqMsg, int num);

private:
    /**
     * @brief elThread evpp客户端。
     */
    evpp::EventLoopThread elThread;
    ClientBaseEVPP base;

    std::string bProtocolVersion = "Version1.1";
    std::string armType = "AR10";

    /**
     * @brief mutexForRegMsg 在信息发送和往队列中注册时上锁。
     */
    std::mutex mutexForRegMsg;
    unsigned long msgSendCount = 0;
    std::queue<MSGReqRegister> msgQueue;

    /**
     * @brief debugPrint 打印更多的调试信息。
     */
    bool debugPrint = false;


    /**
     * @brief 对反馈协议命令对应的数据类型分类。
     */
    std::set<RobotArmCMD> boolDataTypeSet;
    std::set<RobotArmCMD> intDataTypeSet;
    std::set<RobotArmCMD> doubleDataTypeSet;
    std::set<RobotArmCMD> double2DataTypeSet;
    std::set<RobotArmCMD> double6DataTypeSet;
    std::set<RobotArmCMD> double7DataTypeSet;
    std::set<RobotArmCMD> double8DataTypeSet;

};

}
#endif // CLIENTTHREENEW_H

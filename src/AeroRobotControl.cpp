#include <memory>
#include <string>
#include <unistd.h>
#include <cmath>
#include "AeroRobotControl.h"
//#include "ClientOne.h"
//#include "ClientThree.h"
#include "Utiles.h"
#include "arcLog.h"

namespace AeroRobot {

static const double Deg2Rad = M_PI / 180.0;
static const double Rad2Deg = 180.0 / M_PI;


using namespace std;

AeroRobotControl::AeroRobotControl(const char * IP, int robotInfoAcess) :
    //clientTopic_ptr(make_shared<ClientOne>(IP, 13000, false, true)),
    //clientService_ptr(make_shared<ClientThree>(IP, 8008, false, true)),
    serverIP(IP),
//    clientTopicNew_ptr(make_shared<ClientOneNew>(std::string(IP) + ":" + std::to_string(30003), "30003Client")),
//    clientServiceNew_ptr(make_shared<ClientThreeNew>(std::string(IP) + ":" + std::to_string(30001), "30001Client")),
    mRobotInfoAccess(robotInfoAcess),
    version(240402.001)

{}

AeroRobotControl::~AeroRobotControl()
{
    disconnectToRobot();
}


bool AeroRobotControl::connectToRobot()
{
    if(clientTopicNew_ptr && clientServiceNew_ptr){
        if(clientTopicNew_ptr->isConnected() && clientServiceNew_ptr->isConnected()){
            log_ptr->info("机器人链接已经存在。");
            return true;
        }
    }

    /**
     * @brief setSpdLogger 日志初始化。只会被初始化一次。
     */
    setSpdLogger();
    sleep(1);

    log_ptr->info("客户端[版本:{}]准备连接机器人...", version);
    if(clientTopicNew_ptr == nullptr){
        clientTopicNew_ptr = make_shared<ClientOneNew>(std::string(serverIP) + ":" + std::to_string(30003), "30003Client");
    }
    if(!clientTopicNew_ptr->connectToServer()){
        log_ptr->warn("[{}:{}] 机器人服务器30003连接失败。", __FILE__, __LINE__);
        clientTopicNew_ptr.reset();
        return false;
    }
    log_ptr->info("机器人服务器30003连接成功。");

    if(clientServiceNew_ptr == nullptr){
        clientServiceNew_ptr = make_shared<ClientThreeNew>(std::string(serverIP) + ":" + std::to_string(30001), "30001Client");
    }
    if(!clientServiceNew_ptr->connectToServer()){
        log_ptr->warn("[{}:{}] 机器人服务器30001连接失败。", __FILE__, __LINE__);
        clientServiceNew_ptr.reset();
        return false;
    }
    log_ptr->info("机器人服务器30001连接成功。");

    sleep(1);
    log_ptr->info("连接机器人成功。");

    return true;
}

void AeroRobotControl::disconnectToRobot()
{
    if(!clientTopicNew_ptr || !clientServiceNew_ptr){
        log_ptr->info("没有机器人链接需要断开。");
        return;
    }

    if(clientTopicNew_ptr->isConnected()){
        clientTopicNew_ptr->Disconnect();
        clientTopicNew_ptr.reset();
        sleep(1);
        log_ptr->info("断开机器人广播链接成功。");
    }

    if(clientServiceNew_ptr->isConnected()){
        clientServiceNew_ptr->Disconnect();
        clientServiceNew_ptr.reset();
        sleep(1);
        log_ptr->info("断开机器人服务链接成功。");
    }

    log_ptr->info("所有链接都已经断开。");
}



void AeroRobotControl::setEmergencyOn()
{
    clientServiceNew_ptr->setEmergenyOn();
    log_ptr->info("机器人打开急停。");
}
void AeroRobotControl::setEmergencyOff()
{
    clientServiceNew_ptr->setEmergencyOff();
    log_ptr->info("机器人关闭急停。");
}
void AeroRobotControl::setServoOn()
{
    clientServiceNew_ptr->setServeOn();
    log_ptr->info("机器人上伺服。");
}
void AeroRobotControl::setServoOff()
{
    clientServiceNew_ptr->setServeOff();
    log_ptr->info("机器人下伺服。");
}
bool AeroRobotControl::setHomePos()
{
    return clientServiceNew_ptr->setNewHomePos();
}
bool AeroRobotControl::moveToHomePos(double s)
{
    return clientServiceNew_ptr->moveToHome(s);
}
bool AeroRobotControl::moveToZeroPos(double s)
{
    return clientServiceNew_ptr->moveToZero(s);
}



bool AeroRobotControl::getCurrentPoseJointSpace(double * pose)    //double[6]; Unit(rad).
{
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        memcpy(pose, clientTopicNew_ptr->getRobotStatus().JointCurPos, sizeof(double) * 6);
        return true;
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        return clientServiceNew_ptr->getCurrentJoints(pose);
    }
    else{
        return clientServiceNew_ptr->getCurrentJoints(pose);
    }

}
bool AeroRobotControl::getCurrentPoseCartSpace(double * pose)     //double[6]; Unit(m/rad); x,y,z,rz,ry,rx
{
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        memcpy(pose, clientTopicNew_ptr->getRobotStatus().CartTCPPosEuler, sizeof(double) * 6);
        return true;
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        return clientServiceNew_ptr->getCurrentCart(pose);
    }
    else{
        return clientServiceNew_ptr->getCurrentCart(pose);
    }

}
void AeroRobotControl::getCurrentSpeedJointSpace(double * speed)  //double[6]; Unit(rad/s).
{
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        memcpy(speed, clientTopicNew_ptr->getRobotStatus().JointCurVel, sizeof(double) * 6);
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        clientServiceNew_ptr->getCurrentJointsSpeed(speed);
    }
    else{
        clientServiceNew_ptr->getCurrentJointsSpeed(speed);
    }

}
void AeroRobotControl::getCurrentAccelJointSpace(double *accel_ptr)
{
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        memcpy(accel_ptr, clientTopicNew_ptr->getRobotStatus().JointCurAcc, sizeof(double) * 6);
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        clientServiceNew_ptr->getCurrentJointAcce(accel_ptr);
    }
    else{
        clientServiceNew_ptr->getCurrentJointAcce(accel_ptr);
    }
}
void AeroRobotControl::getCurrentAmpereJointSpace(double * Ampere)//double[6]; Unit(A).
{
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        memcpy(Ampere, clientTopicNew_ptr->getRobotStatus().JointCurAmp, sizeof(double) * 6);
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        clientServiceNew_ptr->getCurrentAmpere(Ampere);
    }
    else{
        clientServiceNew_ptr->getCurrentAmpere(Ampere);
    }

}
void AeroRobotControl::getCurrentJointTemperature(double *temper)
{
    clientServiceNew_ptr->getCurrentJointTemperature(temper);
}
bool AeroRobotControl::getDoubleEncoderDiff(double * diff_ptr)
{
    return clientServiceNew_ptr->getDoubleEncoderDiff(diff_ptr);
}
bool AeroRobotControl::isHome()
{
    return clientServiceNew_ptr->isHome();
}
bool AeroRobotControl::isEmergencyOn()
{
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        return clientTopicNew_ptr->getRobotStatus().emergencyState;
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        int err;
        bool eSB = false;
        if(!clientServiceNew_ptr->getRobotEmergencyState(&eSB, err)){
            log_ptr->warn("请求机械臂状态失败[{}]。", err);
        }
        return eSB;
    }
    else{
        int err;
        bool eSB = false;
        if(!clientServiceNew_ptr->getRobotEmergencyState(&eSB, err)){
            log_ptr->warn("请求机械臂状态失败[{}]。", err);
        }
        return eSB;
    }
}
bool AeroRobotControl::isServoOn()
{
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        return clientTopicNew_ptr->getRobotStatus().servoState;
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        int err;
        bool eSB = false;
        if(!clientServiceNew_ptr->getRobotServoState(&eSB, err)){
            log_ptr->warn("请求机械臂状态失败[{}]。", err);
        }
        return eSB;
    }
    else{
        int err;
        bool eSB = false;
        if(!clientServiceNew_ptr->getRobotServoState(&eSB, err)){
            log_ptr->warn("请求机械臂状态失败[{}]。", err);
        }
        return eSB;
    }
}
bool AeroRobotControl::isRunning()
{
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        return clientTopicNew_ptr->getRobotStatus().runningState;
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        int err;
        bool eSB = false;
        if(!clientServiceNew_ptr->getRobotRunningState(&eSB, err)){
            log_ptr->warn("请求机械臂状态失败[{}]。", err);
        }
        return eSB;
    }
    else{
        int err;
        bool eSB = false;
        if(!clientServiceNew_ptr->getRobotRunningState(&eSB, err)){
            log_ptr->warn("请求机械臂状态失败[{}]。", err);
        }
        return eSB;
    }
}
bool AeroRobotControl::getCurrentRobotTCP(std::string & TCPName, std::array<double, 6> & tcpPosEuler)
{
    return clientServiceNew_ptr->getCurrentRobotTCP(TCPName, tcpPosEuler);
}
bool AeroRobotControl::setCurrentRobotTCP(const std::string & TCPName)
{
    return clientServiceNew_ptr->setCurrentRobotTCP(TCPName);
}
bool AeroRobotControl::setNewCurrentRobotTCP(const std::string & TCPName, const std::array<double, 6> & tcpPosEuler)
{
    return clientServiceNew_ptr->setNewCurrentRobotTCP(TCPName, tcpPosEuler);
}



bool AeroRobotControl::MoveContinuousJoint(int axis, bool forwardMove) //axis: [1, 6]. forwardMove: true,false -> forward, backward.
{
    return clientServiceNew_ptr->JogMoveJointSpace(axis-1, forwardMove);
}
bool AeroRobotControl::MoveContinuousCartesianPose(int dir, bool forwardMove, int refCoordinate)//dir: [1, 3] -> x,y,z. forwardMove: true,false -> forward, backward. refCoordinate: 0, 1 -> base, TCP
{
    return clientServiceNew_ptr->JogLineMoveCartesianSpace(dir, forwardMove, refCoordinate);
}
bool AeroRobotControl::MoveContinuousCartesianRota(int dir, bool forwardMove, int refCoordinate)//dir: [1, 3] -> rx,ry,rz. forwardMove: true,false -> forward, backward. refCoordinate: 0, 1 -> base, TCP
{
    return clientServiceNew_ptr->JogRotMoveCartesianSpace(dir, forwardMove, refCoordinate);
}
int AeroRobotControl::getJogSpeedLevel()
{
    return clientTopicNew_ptr->getRobotStatus().JogSpeedLevel;
}
void AeroRobotControl::getJogSpeedVal(double data[3])
{
    memcpy(data, clientTopicNew_ptr->getRobotStatus().JogSpeed, sizeof (double) * 3);
}
void AeroRobotControl::JogSpeedUp()
{
    clientServiceNew_ptr->JogSpeedUp();
}
void AeroRobotControl::JogSpeedDown()
{
    clientServiceNew_ptr->JogSpeedDown();
}
bool AeroRobotControl::JogSpeedDirectSet(int val)
{
    return clientServiceNew_ptr->JogSpeedDirectSet(val);
}
void AeroRobotControl::JogMoveStop()
{
    clientServiceNew_ptr->JogMoveStop();
}



//Point move control.
bool AeroRobotControl::set_csp()          //Set csp mode.
{
   return clientServiceNew_ptr->set_csp();
}
bool AeroRobotControl::isMoveFinished()   //Get pose control motion state.
{
    return clientServiceNew_ptr->isMoveFinished();
}
bool AeroRobotControl::set_joint_move_speed(double s) //Joint space pose control.
{
    return clientServiceNew_ptr->set_joint_move_speed(s);
}
bool AeroRobotControl::moveJointTo(double * tQ)
{
    return clientServiceNew_ptr->moveJointTo(tQ);
}
bool AeroRobotControl::moveJointToWithDefinedSpeed(double * tQ)
{
    return clientServiceNew_ptr->moveJointToWithDefinedSpeed(tQ);
}
bool AeroRobotControl::test(double * tS)
{
    return clientServiceNew_ptr->test(tS);
}
bool AeroRobotControl::moveJointToUntilFinish(double * tQ, double deadline)
{
    bool ifsendsuccess = moveJointTo(tQ);
    if(!ifsendsuccess){
        log_ptr->warn("请求开始moveJointToUntilFinish运动失败。");
        return false;
    }

    double stepTime = 0;
    while(!isMoveFinished()){
        sleep_ms(100);
        stepTime += 0.1;
        if(stepTime > deadline){
            bool ifstopmove = pMoveStop();
            if(!ifstopmove){
                log_ptr->warn("moveJointToUntilFinish运动超时，并且无法让机械臂停止运动。请检查与机械臂的连接情况。");
                return false;
            }
            else{
                log_ptr->warn("moveJointToUntilFinish运动超时，已经请求机械臂提前停止运动，请给机械臂更多的运行等待时间。目前的等待时间是：{}s.", deadline);
                return false;
            }
        }
    }

    int eS = 0, sS = 0;
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        eS = clientTopicNew_ptr->getRobotStatus().emergencyState;
        sS = clientTopicNew_ptr->getRobotStatus().servoState;
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        int err;
        bool eSB = false, sSB = false;
        if(!clientServiceNew_ptr->getRobotEmergencyState(&eSB, err)){
            log_ptr->warn("moveJointToUntilFinish请求机械臂状态失败[{}]。", err);
        }
        if(!clientServiceNew_ptr->getRobotServoState(&sSB, err)){
            log_ptr->warn("moveJointToUntilFinish请求机械臂状态失败[{}]。", err);
        }
        eS = eSB;
        sS = sSB;
    }
    else{
        int err;
        bool eSB = false, sSB = false;
        if(!clientServiceNew_ptr->getRobotEmergencyState(&eSB, err)){
            log_ptr->warn("moveJointToUntilFinish请求机械臂状态失败[{}]。", err);
        }
        if(!clientServiceNew_ptr->getRobotServoState(&sSB, err)){
            log_ptr->warn("moveJointToUntilFinish请求机械臂状态失败[{}]。", err);
        }
        eS = eSB;
        sS = sSB;
    }

    if(eS || !sS){
        log_ptr->warn("moveJointToUntilFinish停止运动，发现机械臂状态异常：EM[{}], SERVO[{}].", eS, sS);
        return false;
    }

    return true;
}
bool AeroRobotControl::set_Task_move_speed(const double * s)//Cartesian space pose control.
{
    return clientServiceNew_ptr->set_Task_move_speed(s);
}
bool AeroRobotControl::moveTaskTo(double * tC)
{
    return clientServiceNew_ptr->moveTaskTo(tC);
}
bool AeroRobotControl::moveTaskBy(double * tC)
{
    return clientServiceNew_ptr->moveTaskBy(tC);
}
bool AeroRobotControl::moveTaskByTCP(double * tC)
{
    return clientServiceNew_ptr->moveTaskByTCP(tC);
}
bool AeroRobotControl::moveTaskToWithSpeed(double * tC)
{
    return clientServiceNew_ptr->moveTaskToWithSpeed(tC);
}
bool AeroRobotControl::moveTaskToWithDefinedSpeed(double * tC)
{
    return clientServiceNew_ptr->moveTaskToWithDefinedSpeed(tC);
}
bool AeroRobotControl::moveTaskToUntilFinish(double * tC, double deadline)
{
    bool ifsendsuccess = moveTaskTo(tC);
    if(!ifsendsuccess){
        log_ptr->warn("请求开始moveTaskToUntilFinish运动失败。");
        return false;
    }

    double time =0;
    while(!isMoveFinished()){
        sleep_ms(100);
        time += 0.1;
        if(time > deadline){
            bool ifstopmove = pMoveStop();
            if(!ifstopmove){
                log_ptr->warn("moveTaskToUntilFinish运动超时，并且无法让机械臂停止运动。请检查与机械臂的连接情况。");
                return false;
            }
            else{
                log_ptr->warn("moveTaskToUntilFinish运动超时，已经请求机械臂提前停止运动，请给机械臂更多的运行等待时间。目前的等待时间是：{}s.", deadline);
                return false;
            }
        }
    }

    int eS = 0, sS = 0;
    if(mRobotInfoAccess == INFO_BROADCAST_WAY){
        eS = clientTopicNew_ptr->getRobotStatus().emergencyState;
        sS = clientTopicNew_ptr->getRobotStatus().servoState;
    }
    else if(mRobotInfoAccess == INFO_SERVICE_WAY){
        int err;
        bool eSB = false, sSB = false;
        if(!clientServiceNew_ptr->getRobotEmergencyState(&eSB, err)){
            log_ptr->warn("moveTaskToUntilFinish请求机械臂状态失败[{}]。", err);
        }
        if(!clientServiceNew_ptr->getRobotServoState(&sSB, err)){
            log_ptr->warn("moveTaskToUntilFinish请求机械臂状态失败[{}]。", err);
        }
        eS = eSB;
        sS = sSB;
    }
    else{
        int err;
        bool eSB = false, sSB = false;
        if(!clientServiceNew_ptr->getRobotEmergencyState(&eSB, err)){
            log_ptr->warn("moveTaskToUntilFinish请求机械臂状态失败[{}]。", err);
        }
        if(!clientServiceNew_ptr->getRobotServoState(&sSB, err)){
            log_ptr->warn("moveTaskToUntilFinish请求机械臂状态失败[{}]。", err);
        }
        eS = eSB;
        sS = sSB;
    }

    if(eS || !sS){
        log_ptr->warn("moveTaskToUntilFinish停止运动，发现机械臂状态异常：EM[{}], SERVO[{}].", eS, sS);
        return false;
    }

    return true;
}
bool AeroRobotControl::pMoveStop()
{
    return clientServiceNew_ptr->moveStop();
}



bool AeroRobotControl::moveJointToContinuesWithSpeed(double *tJ)
{
    return clientServiceNew_ptr->moveJointToContinueswithSpeed(tJ);
}
bool AeroRobotControl::moveJointToContinuesWithDurat(double *tJ)
{
    return clientServiceNew_ptr->moveJointToContinueswithDurat(tJ);
}
bool AeroRobotControl::moveTaskToContinues(double *tC)
{
    return clientServiceNew_ptr->moveTaskToContinues(tC);
}


bool AeroRobotControl::set_csv()                           //Set csv mode.
{
    return clientServiceNew_ptr->set_csv();
}
bool AeroRobotControl::moveJointSpeedTo(const double * ts) //Joint space speed control.
{
    return clientServiceNew_ptr->moveJointSpeedTo(ts);
}



bool AeroRobotControl::complexWayPointsPushBack(double *tW)
{
    return clientServiceNew_ptr->complexWayPointsPushBack(tW);
}
bool AeroRobotControl::complexWayPointsClear()
{
    return clientServiceNew_ptr->complexWayPointsClear();
}
bool AeroRobotControl::complexWayPointsRun()
{
    return clientServiceNew_ptr->complexWayPointsRun();
}
bool AeroRobotControl::complexWayPointsRunIsFinish(bool & ret)
{
    return clientServiceNew_ptr->complexWayPointsRunIsFinish(ret);
}
bool AeroRobotControl::pushLineSerialPoints(double * tW, double LineSpeed)
{
    return clientServiceNew_ptr->pushLineSerialPoints(tW, LineSpeed);
}
bool AeroRobotControl::moveTaskToLineSerial()
{
    return clientServiceNew_ptr->moveTaskToLineSerial();
}
bool AeroRobotControl::moveTaskToLineSerialPlan()
{
    return clientServiceNew_ptr->moveTaskToLineSerialPlan();
}
bool AeroRobotControl::moveTaskToLineSerialRun()
{
    return clientServiceNew_ptr->moveTaskToLineSerialRun();
}
bool AeroRobotControl::clearLineSerialPoints()
{
    return clientServiceNew_ptr->clearLineSerialPoints();
}
bool AeroRobotControl::openProg(const std::string fileName, int &line, int &error)
{
    bool ret = clientServiceNew_ptr->openProg(fileName, error);
    if(!ret){
        line = -1;
        log_ptr->warn("打开程序失败。");
        return false;
    }

    bool ret2 = clientServiceNew_ptr->updatePoints(line, error);

    return ret && ret2;
}
bool AeroRobotControl::runProg(int &error)
{
    return clientServiceNew_ptr->runProg(error);
}
bool AeroRobotControl::stopProg(int &error)
{
    return clientServiceNew_ptr->stopProg(error);
}
bool AeroRobotControl::saveProg(int &error)
{
    return clientServiceNew_ptr->saveProg(error);
}
bool AeroRobotControl::saveExitProg(int &error)
{
    return clientServiceNew_ptr->saveExitProg(error);
}
bool AeroRobotControl::exitProg(int &error)
{
    return clientServiceNew_ptr->exitProg(error);
}
bool AeroRobotControl::pushBackPoint(const ProgramWayPoint & w, int &error)
{
    return clientServiceNew_ptr->pushBackPoint(w, error);
}
bool AeroRobotControl::insertPoint(const ProgramWayPoint & w, int &error)
{
    return clientServiceNew_ptr->insertPoint(w, error);
}
bool AeroRobotControl::erasePoint(int index, int &error)
{
    return clientServiceNew_ptr->erasePoint(index, error);
}
bool AeroRobotControl::replacePoint(const ProgramWayPoint & w, int &error)
{
    return clientServiceNew_ptr->replacePoint(w, error);
}
bool AeroRobotControl::clearAllPoints(int &error)
{
    return clientServiceNew_ptr->clearAllPoints(error);
}

bool AeroRobotControl::preJointsMoveTo(double * tq, int & err)
{
    return clientServiceNew_ptr->preJointsMoveTo(tq, err);
}
bool AeroRobotControl::preJointsStop(int & err)
{
    return clientServiceNew_ptr->preJointsStop(err);
}
bool AeroRobotControl::preJointsServoOn(int & err)
{
    return clientServiceNew_ptr->preJointsServoOn(err);
}
bool AeroRobotControl::preJointsServoOff(int & err)
{
    return clientServiceNew_ptr->preJointsServoOff(err);
}
bool AeroRobotControl::preJointsIsMoveFinished(int & err)
{
    return clientServiceNew_ptr->preJointsIsMoveFinished(err);
}
bool AeroRobotControl::preJointsGetPos(double * pos_ptr, int &err)
{
    return clientServiceNew_ptr->preJointsGetPos(pos_ptr, err);
}
bool AeroRobotControl::preJointGetVel(double * vel_ptr, int &err)
{
    return clientServiceNew_ptr->preJointsGetVel(vel_ptr, err);
}
bool AeroRobotControl::preJointsGetTorque(double * torque_ptr, int &err)
{
    return clientServiceNew_ptr->preJointsGetTor(torque_ptr, err);
}

bool AeroRobotControl::afterJointsMoveTo(double * tq, int & err)
{
    return clientServiceNew_ptr->afterJointsMoveTo(tq, err);
}
bool AeroRobotControl::afterJointsStop(int & err)
{
    return clientServiceNew_ptr->afterJointsStop(err);
}
bool AeroRobotControl::afterJointsServoOn(int & err)
{
    return clientServiceNew_ptr->afterJointsServoOn(err);
}
bool AeroRobotControl::afterJointsServoOff(int & err)
{
    return clientServiceNew_ptr->afterJointsServoOff(err);
}
bool AeroRobotControl::afterJointsIsMoveFinished(int & err)
{
    return clientServiceNew_ptr->afterJointsIsMoveFinished(err);
}
bool AeroRobotControl::afterJointsGetPos(double * pos_ptr, int &err)
{
    return clientServiceNew_ptr->afterJointsGetPos(pos_ptr, err);
}
bool AeroRobotControl::afterJointGetVel(double * vel_ptr, int &err)
{
    return clientServiceNew_ptr->afterJointsGetVel(vel_ptr, err);
}
bool AeroRobotControl::afterJointsGetTorque(double * torque_ptr, int &err)
{
    return clientServiceNew_ptr->afterJointsGetTor(torque_ptr, err);
}

bool AeroRobotControl::connectToSixDForce()
{
    return clientServiceNew_ptr->connectToSixDForce();
}
bool AeroRobotControl::disConnectToSixDForce()
{
    return clientServiceNew_ptr->disConnectToSixDForce();
}
bool AeroRobotControl::getSixDForceData(double * data_ptr)
{
    return clientServiceNew_ptr->getSixDForceData(data_ptr);
}

bool AeroRobotControl::getCartPosFromJoints(double * tarJoints, double * cartGot, int & err)
{
    return  clientServiceNew_ptr->getCartPosFromJoints(tarJoints, cartGot, err);
}


bool AeroRobotControl::getJointsPosFromCartPos(double * tarCart, double * jointsGot, int & err)
{
    return  clientServiceNew_ptr->getJointsPosFromCartPos(tarCart, jointsGot, err);
}


void AeroRobotControl::isPrintingError(bool p)
{
    //clientTopicNew_ptr->isDebugPrint(p);
    clientServiceNew_ptr->isDebugPrint(p);
}

void AeroRobotControl::isBroadcastPrint(bool p)
{
    clientTopicNew_ptr->isDebugPrint(p);
}




//bool AeroRobotControl::getRobotStatusPtrRegister(RobotStatus * userData_ptr)
//{
//    return clientTopic_ptr->getRobotStatusPtrRegister(userData_ptr);
//}


//void AeroRobotControl::getRobotState(RobotState & robotState)
//{
//    clientTopic_ptr->getRobotState(robotState);
//}

//void AeroRobotControl::changeBroadcastCycTime(int time)
//{
//    clientTopic_ptr->changeBroadcastCycTime(time);
//}
//void AeroRobotControl::MoveInchingCartesianPos(int refCoor, int dir, bool fMove)
//{
//    clientTopic_ptr->moveInchingPos(refCoor,dir, fMove);
//}

//void AeroRobotControl::MoveInchingCartesianRot(int refCoor, int dir, bool fMove)
//{
//    clientTopic_ptr->moveInchingRot(refCoor,dir, fMove);
//}

//double AeroRobotControl::getCurrentInchingPosLevel()
//{
//    const RobotStatus &c = clientTopic_ptr->getRobotStatus();
//    return c.JogInchingPose / 1000;
//}

//double AeroRobotControl::getCurrentInchingRotLevel()
//{
//    const RobotStatus &c = clientTopic_ptr->getRobotStatus();
//    return c.JogInchingEuler * M_PI / 180;
//}


}

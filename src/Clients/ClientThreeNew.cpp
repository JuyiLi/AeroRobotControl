#include "ClientThreeNew.h"
#include "spdlog/spdlog.h"
#include "arcLog.h"
#include "Utiles.h"

using namespace std;

namespace AeroRobot {


//================================================== 初始化函数 ============================================================

ClientThreeNew::ClientThreeNew(const std::string& serverAddr, const std::string& name) : base(elThread.loop(), serverAddr, name)
{
    base.SetMessageCallback(std::bind(&ClientThreeNew::protocolProcess, this, std::placeholders::_1));


    boolDataTypeSet.insert(CMD_EMERGENCY_STOP);
    boolDataTypeSet.insert(CMD_SET_SERVO);
    boolDataTypeSet.insert(CMD_SET_BRAKE);
    boolDataTypeSet.insert(CMD_STOP);
    boolDataTypeSet.insert(CMD_SET_HOME_POS);
    boolDataTypeSet.insert(CMD_SET_CSP_MODE);
    boolDataTypeSet.insert(CMD_SET_CSV_MODE);
    boolDataTypeSet.insert(CMD_IS_ROBOT_RUNNING);
    boolDataTypeSet.insert(CMD_IS_MOVE_FINISEHD);
    boolDataTypeSet.insert(CMD_IS_HOME);
    boolDataTypeSet.insert(CMD_PROGRAM_SAVE);
    boolDataTypeSet.insert(CMD_PROGRAM_SAVE_EXIT);
    boolDataTypeSet.insert(CMD_PROGRAM_EXIT);
    boolDataTypeSet.insert(CMD_PROGRAM_RUN);
    boolDataTypeSet.insert(CMD_PROGRAM_STOP);
    boolDataTypeSet.insert(CMD_WAYPOINT_CLEAR);
    boolDataTypeSet.insert(CMD_COMPLEX_WAYPOINTS_CLEAR);
    boolDataTypeSet.insert(CMD_COMPLEX_WAYPOINTS_RUN);
    boolDataTypeSet.insert(CMD_MOVE_HOME);
    boolDataTypeSet.insert(CMD_MOVE_ZERO);
    boolDataTypeSet.insert(CMD_SET_JOINT_MOVE_SPEED);
    boolDataTypeSet.insert(CMD_SET_CART_MOVE_SPEED);
    boolDataTypeSet.insert(CMD_SET_HOME_POS);
    boolDataTypeSet.insert(CMD_JOINT_MOVE_TO);
    boolDataTypeSet.insert(CMD_JOINT_MOVE_BY);
    boolDataTypeSet.insert(CMD_TASK_MOVE_TO);
    boolDataTypeSet.insert(CMD_TASK_MOVE_BY);
    boolDataTypeSet.insert(CMD_TASK_MOVE_BY_TCP);
    boolDataTypeSet.insert(CMD_JOINTSPEED_MOVE_TO);
    boolDataTypeSet.insert(CMD_COMPLEX_WAYPOINTS_PUSH);
    boolDataTypeSet.insert(CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED);
    boolDataTypeSet.insert(CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT);
    boolDataTypeSet.insert(CMD_TASK_MOVE_TO_SPEED);
    boolDataTypeSet.insert(CMD_TASK_MOVE_TO_CONTINUOUS);
    boolDataTypeSet.insert(CMD_JOG_MOVE);
    boolDataTypeSet.insert(CMD_JOG_SPEED_SET);
    boolDataTypeSet.insert(CMD_JOG_SPEED_DSET);
    boolDataTypeSet.insert(CMD_CONNECT_TO_SIXFORCE);
    boolDataTypeSet.insert(CMD_TEST);
    boolDataTypeSet.insert(CMD_COMPLEX_LINE_WP_PUSH);
    boolDataTypeSet.insert(CMD_COMPLEX_LINE_WP_CLEAR);
    boolDataTypeSet.insert(CMD_COMPLEX_LINE_WP_PLAN_RUN);
    boolDataTypeSet.insert(CMD_COMPLEX_LINE_WP_PLAN);
    boolDataTypeSet.insert(CMD_COMPLEX_LINE_WP_RUN);
    boolDataTypeSet.insert(CMD_JOINT_MOVE_TO_WITH_DEFINED_SPEED);
    boolDataTypeSet.insert(CMD_TASK_MOVE_TO_WITH_DEFINED_SPEED);


    intDataTypeSet.insert(CMD_WAYPOINT_UPDATE);

    double6DataTypeSet.insert(CMD_GET_JOINT_POSITION);
    double6DataTypeSet.insert(CMD_GET_JOINT_VELOCITY);
    double6DataTypeSet.insert(CMD_GET_CART_POSITION);
    double6DataTypeSet.insert(CMD_GET_CART_VELOCITY);
    double6DataTypeSet.insert(CMD_GET_TORQUE);
    double6DataTypeSet.insert(CMD_GET_JOINT_ACCEL);
    double6DataTypeSet.insert(CMD_GET_TEMPERATURE);
    double6DataTypeSet.insert(CMD_GET_DOUBLE_ENCODER_DIFFERENCE);
    double6DataTypeSet.insert(CMD_GET_SIXFORCE_DATA);
    double6DataTypeSet.insert(CMD_GET_CARTPOS_FROM_JOINT);
    double6DataTypeSet.insert(CMD_GET_JOINTPOS_FROM_CART);





}

bool ClientThreeNew::connectToServer()
{
    elThread.Start(true);
    base.Connect();
    usleep(100000);

    double s = 0;
    while (s <= 1) {
        usleep(100000);
        if(base.isConnected()){
            return true;
        }
        s += 0.1;
    }

    if(log_ptr){
        log_ptr->warn("等待时间超时1S，连接服务器失败。");
        Disconnect();
    }
    return false;
}

void ClientThreeNew::Disconnect()
{
    base.Disconnect();
    usleep(1000);
    elThread.Stop(true);
}

bool ClientThreeNew::isConnected()
{
    return base.isConnected();
}


//=======================================================================================================================


//================================================= 业务请求函数 ===========================================================

bool ClientThreeNew::setEmergenyOn()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_EMERGENCY_STOP, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::setEmergencyOff()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = false;
    if(!requestServer(CMD_EMERGENCY_STOP, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::setServeOn()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_SET_SERVO, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::setServeOff()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = false;
    if(!requestServer(CMD_SET_SERVO, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveToHome(double s)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_MOVE_HOME, DATATYPE_ENUM_DOUBLE, &s, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveToZero(double s)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_MOVE_ZERO, DATATYPE_ENUM_DOUBLE, &s, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::setNewHomePos()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_SET_HOME_POS, DATATYPE_ENUM_DOUBLE, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::isHome()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_IS_HOME, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::getCurrentJoints(double * joints)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_JOINT_POSITION, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(joints, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(joints, resData.double6, 6 * sizeof(double));
        return true;
    }
}

bool ClientThreeNew::getCurrentCart(double * pose)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_CART_POSITION, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(pose, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(pose, resData.double6, 6 * sizeof(double));
        return true;
    }
}

bool ClientThreeNew::getCurrentJointsSpeed(double * speed)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_JOINT_VELOCITY, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(speed, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(speed, resData.double6, 6 * sizeof(double));
        return true;
    }
}

bool ClientThreeNew::getCurrentJointAcce(double *acce)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_JOINT_ACCEL, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(acce, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(acce, resData.double6, 6 * sizeof(double));
        return true;
    }
}

bool ClientThreeNew::getCurrentJointTemperature(double *temper)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_TEMPERATURE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(temper, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(temper, resData.double6, 6 * sizeof(double));
        return true;
    }
}

bool ClientThreeNew::getCurrentAmpere(double * Ampere)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_TORQUE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(Ampere, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(Ampere, resData.double6, 6 * sizeof(double));
        return true;
    }
}

bool ClientThreeNew::getDoubleEncoderDiff(double * diff_ptr)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_DOUBLE_ENCODER_DIFFERENCE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(diff_ptr, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(diff_ptr, resData.double6, 6 * sizeof(double));
        return true;
    }
}
bool ClientThreeNew::getRobotEmergencyState(bool * state_ptr, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_GET_EMERGENCY_STATE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        *state_ptr = resData.boolval;
        return true;
    }
}
bool ClientThreeNew::getRobotServoState(bool * state_ptr, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_GET_SERVO_STATE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        *state_ptr = resData.boolval;
        return true;
    }
}
bool ClientThreeNew::getRobotRunningState(bool * state_ptr, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_GET_RUNNING_STATE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        *state_ptr = resData.boolval;
        return true;
    }
}

bool ClientThreeNew::getCartPosFromJoints(double * tarJoints, double * cartGot, int & err)
{
    robotarmServiceData resData;
    if(!requestServer(CMD_GET_CARTPOS_FROM_JOINT, DATATYPE_ENUM_DOUBLE6, tarJoints, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(cartGot, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(cartGot, resData.double6, 6 * sizeof(double));
        return true;
    }
}
bool ClientThreeNew::getJointsPosFromCartPos(double * tarCart, double * jointsGot, int & err)
{
    robotarmServiceData resData;
    if(!requestServer(CMD_GET_JOINTPOS_FROM_CART, DATATYPE_ENUM_DOUBLE6, tarCart, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(jointsGot, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(jointsGot, resData.double6, 6 * sizeof(double));
        return true;
    }
}

bool ClientThreeNew::set_csp()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_SET_CSP_MODE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::isMoveFinished()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_IS_MOVE_FINISEHD, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::set_joint_move_speed(double s)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_SET_JOINT_MOVE_SPEED, DATATYPE_ENUM_DOUBLE, &s, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveJointTo(double * tQ)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_JOINT_MOVE_TO, DATATYPE_ENUM_DOUBLE6, tQ, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveJointToWithDefinedSpeed(double *tQ)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_JOINT_MOVE_TO_WITH_DEFINED_SPEED, DATATYPE_ENUM_DOUBLE7, tQ, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::test(double * tS)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_TEST, DATATYPE_ENUM_DOUBLE7, tS, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveJointBy(double * tQ)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_JOINT_MOVE_BY, DATATYPE_ENUM_DOUBLE6, tQ, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::set_Task_move_speed(const double * s)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_SET_CART_MOVE_SPEED, DATATYPE_ENUM_DOUBLE2, s, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveTaskTo(double * tC)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_TASK_MOVE_TO, DATATYPE_ENUM_DOUBLE6, tC, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveTaskToContinues(double * tC)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_TASK_MOVE_TO_CONTINUOUS, DATATYPE_ENUM_DOUBLE8, tC, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveJointToContinueswithSpeed(double * tC)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED, DATATYPE_ENUM_DOUBLE7, tC, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveJointToContinueswithDurat(double * tC)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT, DATATYPE_ENUM_DOUBLE7, tC, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveTaskBy(double * tC)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_TASK_MOVE_BY, DATATYPE_ENUM_DOUBLE6, tC, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveTaskByTCP(double * tC)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_TASK_MOVE_BY_TCP, DATATYPE_ENUM_DOUBLE6, tC, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveTaskToWithSpeed(double * tC)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_TASK_MOVE_TO_SPEED, DATATYPE_ENUM_DOUBLE8, tC, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveTaskToWithDefinedSpeed(double *tC)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_TASK_MOVE_TO_WITH_DEFINED_SPEED, DATATYPE_ENUM_DOUBLE8, tC, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveStop()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_STOP, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::set_csv()
{   
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_SET_CSV_MODE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::moveJointSpeedTo(const double * ts)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_JOINTSPEED_MOVE_TO, DATATYPE_ENUM_DOUBLE6, ts, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::JogMoveJointSpace(int axis, bool forwardMove) //axis: [0, 5]. forwardMove: true,false -> forward, backward.
{
    robotarmServiceData resData;
    int err = 0;
    int data[3];
    data[0] = 0;
    data[1] = axis;
    if(forwardMove){
        data[2] = 1;
    }
    else{
        data[2] = -1;
    }
    if(!requestServer(CMD_JOG_MOVE, DATATYPE_ENUM_INT3, data, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }

}

bool ClientThreeNew::JogLineMoveCartesianSpace(int dir, bool forwardMove, int refCoordinate)//dir: [1, 3] ： x,y,z.； forwardMove: true,false : forward, backward.。refCoordinate: 0, 1 ： base, TCP
{
    robotarmServiceData resData;
    int err = 0;
    int data[3];
    data[0] = 1;
    if(dir == 1){
        if(refCoordinate == 0){
            data[1] = 7;
        }
        else if(refCoordinate == 1){
            data[1] = 13;
        }
        else{

        }
    }
    else if(dir == 2){
        if(refCoordinate == 0){
            data[1] = 8;
        }
        else if(refCoordinate == 1){
            data[1] = 14;
        }
        else{

        }
    }
    else if(dir == 3){
        if(refCoordinate == 0){
            data[1] = 9;
        }
        else if(refCoordinate == 1){
            data[1] = 15;
        }
        else{

        }
    }
    else{

    }

    if(forwardMove){
        data[2] = 1;
    }
    else{
        data[2] = -1;
    }
    if(!requestServer(CMD_JOG_MOVE, DATATYPE_ENUM_INT3, data, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::JogRotMoveCartesianSpace(int dir, bool forwardMove, int refCoordinate)//dir: [1, 3] -> rx,ry,rz； forwardMove: true,false : forward, backward.。refCoordinate: 0, 1 ： base, TCP
{
    robotarmServiceData resData;
    int err = 0;
    int data[3];
    data[0] = 1;
    if(dir == 1){
        if(refCoordinate == 0){
            data[1] = 10;
        }
        else if(refCoordinate == 1){
            data[1] = 16;
        }
        else{

        }
    }
    else if(dir == 2){
        if(refCoordinate == 0){
            data[1] = 11;
        }
        else if(refCoordinate == 1){
            data[1] = 17;
        }
        else{

        }
    }
    else if(dir == 3){
        if(refCoordinate == 0){
            data[1] = 12;
        }
        else if(refCoordinate == 1){
            data[1] = 18;
        }
        else{

        }
    }
    else{

    }

    if(forwardMove){
        data[2] = 1;
    }
    else{
        data[2] = -1;
    }
    if(!requestServer(CMD_JOG_MOVE, DATATYPE_ENUM_INT3, data, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::JogSpeedUp()
{
    robotarmServiceData resData;
    int err = 0;
    int c = 1;
    if(!requestServer(CMD_JOG_SPEED_SET, DATATYPE_ENUM_INT, &c, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::JogSpeedDown()
{
    robotarmServiceData resData;
    int err = 0;
    int c = -1;
    if(!requestServer(CMD_JOG_SPEED_SET, DATATYPE_ENUM_INT, &c, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::JogSpeedDirectSet(int val)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_JOG_SPEED_DSET, DATATYPE_ENUM_INT, &val, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::JogMoveStop()
{
    robotarmServiceData resData;
    int err = 0;
    int data[3];
    data[0] = 2;
    if(!requestServer(CMD_JOG_MOVE, DATATYPE_ENUM_INT3, data, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::preJointsMoveTo(double * tq, int & err)
{
    robotarmServiceData resData;
    if(!requestServer(CMD_PREJOINTS_MOVE_TO, DATATYPE_ENUM_DOUBLE7, tq, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::preJointsStop(int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PREJOINTS_STOP, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::preJointsServoOn(int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PREJOINTS_SERVO, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::preJointsServoOff(int & err)
{
    robotarmServiceData resData;
    bool req = false;
    if(!requestServer(CMD_PREJOINTS_SERVO, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::preJointsIsMoveFinished(int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PREJOINTS_ISFINISHED, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::preJointsGetPos(double * pos, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PREJOINTS_GET_POSITION, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(pos, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(pos, resData.double6, 6 * sizeof(double));
        return true;
    }
}
bool ClientThreeNew::preJointsGetVel(double * vel, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PREJOINTS_GET_VELOCITY, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(vel, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(vel, resData.double6, 6 * sizeof(double));
        return true;
    }
}
bool ClientThreeNew::preJointsGetTor(double * tor, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PREJOINTS_GET_TORQUE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(tor, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(tor, resData.double6, 6 * sizeof(double));
        return true;
    }
}

bool ClientThreeNew::afterJointsMoveTo(double * tq, int & err)
{
    robotarmServiceData resData;
    if(!requestServer(CMD_AFTERJOINTS_MOVE_TO, DATATYPE_ENUM_DOUBLE7, tq, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::afterJointsStop(int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_AFTERJOINTS_STOP, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::afterJointsServoOn(int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_AFTERJOINTS_SERVO, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::afterJointsServoOff(int & err)
{
    robotarmServiceData resData;
    bool req = false;
    if(!requestServer(CMD_AFTERJOINTS_SERVO, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::afterJointsIsMoveFinished(int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_AFTERJOINTS_ISFINISHED, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::afterJointsGetPos(double * pos, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_AFTERJOINTS_GET_POSITION, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(pos, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(pos, resData.double6, 6 * sizeof(double));
        return true;
    }
}
bool ClientThreeNew::afterJointsGetVel(double * vel, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_AFTERJOINTS_GET_VELOCITY, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(vel, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(vel, resData.double6, 6 * sizeof(double));
        return true;
    }
}
bool ClientThreeNew::afterJointsGetTor(double * tor, int & err)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_AFTERJOINTS_GET_TORQUE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(tor, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(tor, resData.double6, 6 * sizeof(double));
        return true;
    }
}


bool ClientThreeNew::getCurrentRobotTCP(std::string & TCPName, std::array<double, 6> & tcpPosEuler)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_CURRENT_TCP, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        TCPName = std::string(resData.robotTCPInfo.RobotTCPName);
        for(size_t i = 0; i < 6; ++i){
            tcpPosEuler[i] = resData.robotTCPInfo.RobotPosEuler[i];
        }
        return true;
    }
}

bool ClientThreeNew::setCurrentRobotTCP(const std::string & TCPName)
{
    if(TCPName.size() > SIZE_OF_CHAR_ARRAY20_MAX){
        spdlog::warn("{}:{} TCP名字大于20Byte，请重新出入。", __FILE__, __LINE__);
        return false;
    }

    robotarmServiceData resData;
    int err = 0;
    char name[20];
    memset(name, 0, 20);
    memcpy(name, TCPName.c_str(), TCPName.size());
    if(!requestServer(CMD_SET_CURRENT_TCP, DATATYPE_ENUM_CHARARRAY20, name, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::setNewCurrentRobotTCP(const std::string & TCPName, const std::array<double, 6> & tcpPosEuler)
{
    if(TCPName.size() > SIZE_OF_CHAR_ARRAY20_MAX){
        spdlog::warn("{}:{} TCP名字大于20Byte，请重新出入。", __FILE__, __LINE__);
        return false;
    }

    RobotTCPInfo tmpTCP;
    memset(&tmpTCP, 0, SIZE_OF_ROBOT_TCP_INFO);
    strcpy(tmpTCP.RobotTCPName, TCPName.c_str());
    for(size_t i = 0; i < 6; ++i){
        tmpTCP.RobotPosEuler[i] = tcpPosEuler[i];
    }

    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_SET_NEW_TO_CURRENT_TCP, DATATYPE_ENUM_ROBOTTCP, &tmpTCP, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::complexWayPointsPushBack(double * tW)
{
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_COMPLEX_WAYPOINTS_PUSH, DATATYPE_ENUM_DOUBLE6, tW, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::complexWayPointsClear()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_COMPLEX_WAYPOINTS_CLEAR, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::complexWayPointsRun()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_COMPLEX_WAYPOINTS_RUN, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::complexWayPointsRunIsFinish(bool & ret)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_COMPLEX_WS_IS_FINISHED, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        ret = resData.boolval;
        return true;
    }
}

bool ClientThreeNew::pushLineSerialPoints(double * tW, double LineSpeed)
{
    ProgramWayPoint t;
    memset(&t, 0, SIZE_OF_PROGRAM_WAYPOINT);
    for(size_t i = 0; i < 6; ++i){
        t.pointData[i] = tW[i];
    }
    t.speedForLineMotion[0] = LineSpeed;
    t.speedForLineMotion[1] = 0.05;

    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_COMPLEX_LINE_WP_PUSH, DATATYPE_ENUM_WAYPOINT, &t, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::moveTaskToLineSerial()
{
    bool ret = true;
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_COMPLEX_LINE_WP_PLAN_RUN, DATATYPE_ENUM_BOOL, &ret, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::moveTaskToLineSerialPlan()
{
    bool ret = true;
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_COMPLEX_LINE_WP_PLAN, DATATYPE_ENUM_BOOL, &ret, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::moveTaskToLineSerialRun()
{
    bool ret = true;
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_COMPLEX_LINE_WP_RUN, DATATYPE_ENUM_BOOL, &ret, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::clearLineSerialPoints()
{
    bool ret = true;
    robotarmServiceData resData;
    int err = 0;
    if(!requestServer(CMD_COMPLEX_LINE_WP_CLEAR, DATATYPE_ENUM_BOOL, &ret, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::newProg(const std::string fileName, int &error)
{
    if(fileName.size() > SIZE_OF_CHAR_ARRAY100_MAX){
        spdlog::warn("{}:{} 程序名字大于100Byte，请重新出入。", __FILE__, __LINE__);
        return false;
    }

    robotarmServiceData resData;
    if(!requestServer(CMD_PROGRAM_NEW, DATATYPE_ENUM_CHARARRAY100, fileName.c_str(), &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::openProg(const std::string fileName, int &error)
{
    if(fileName.size() > SIZE_OF_CHAR_ARRAY100_MAX){
        spdlog::warn("{}:{} 程序名字大于100Byte，请重新出入。", __FILE__, __LINE__);
        return false;
    }

    robotarmServiceData resData;
    if(!requestServer(CMD_PROGRAM_OPEN, DATATYPE_ENUM_CHARARRAY100, fileName.c_str(), &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::runProg(int &error)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PROGRAM_RUN, DATATYPE_ENUM_BOOL, &req, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::stopProg(int &error)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PROGRAM_STOP, DATATYPE_ENUM_BOOL, &req, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::saveProg(int &error)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PROGRAM_SAVE, DATATYPE_ENUM_BOOL, &req, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::saveExitProg(int &error)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PROGRAM_SAVE_EXIT, DATATYPE_ENUM_BOOL, &req, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::exitProg(int &error)
{
    robotarmServiceData resData;
    bool req = true;
    if(!requestServer(CMD_PROGRAM_EXIT, DATATYPE_ENUM_BOOL, &req, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::pushBackPoint(const ProgramWayPoint & w, int &error)
{
    robotarmServiceData resData;
    if(!requestServer(CMD_WAYPOINT_PUSHBACK, DATATYPE_ENUM_WAYPOINT, &w, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::insertPoint(const ProgramWayPoint & w, int &error)
{
    robotarmServiceData resData;
    if(!requestServer(CMD_WAYPOINT_INSERT, DATATYPE_ENUM_WAYPOINT, &w, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::erasePoint(int index, int &error)
{
    robotarmServiceData resData;
    if(!requestServer(CMD_WAYPOINT_ERASE, DATATYPE_ENUM_INT, &index, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::replacePoint(const ProgramWayPoint & w, int &error)
{
    robotarmServiceData resData;
    if(!requestServer(CMD_WAYPOINT_REPLACE, DATATYPE_ENUM_WAYPOINT, &w, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::clearAllPoints(int &error)
{
    robotarmServiceData resData;
    bool ret = true;
    if(!requestServer(CMD_WAYPOINT_CLEAR, DATATYPE_ENUM_BOOL, &ret, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}

bool ClientThreeNew::updatePoints(int & line, int &error)
{
    robotarmServiceData resData;
    bool ret = true;
    if(!requestServer(CMD_WAYPOINT_UPDATE, DATATYPE_ENUM_BOOL, &ret, &resData, &error)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        line = -1;
        return false;
    }
    else{
        line = resData.intval;
        return true;
    }
}


bool ClientThreeNew::connectToSixDForce()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_CONNECT_TO_SIXFORCE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::disConnectToSixDForce()
{
    robotarmServiceData resData;
    int err = 0;
    bool req = false;
    if(!requestServer(CMD_CONNECT_TO_SIXFORCE, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        return false;
    }
    else{
        return resData.boolval;
    }
}
bool ClientThreeNew::getSixDForceData(double * data_ptr)
{
    robotarmServiceData resData;
    int err = 0;
    bool req = true;
    if(!requestServer(CMD_GET_SIXFORCE_DATA, DATATYPE_ENUM_BOOL, &req, &resData, &err)){
        if(log_ptr){
            log_ptr->warn("请求数据失败。");
        }
        memset(data_ptr, 0, 6 * sizeof(double));
        return false;
    }
    else{
        memcpy(data_ptr, resData.double6, 6 * sizeof(double));
        return true;
    }
}

//=======================================================================================================================


//================================================= 通讯底层函数 ===========================================================

/**
 * @brief 主动发送数据的函数
 * @return
 */
bool ClientThreeNew::requestServer(RobotArmCMD cmdId, DataTypeEnum reqDataType, const void *requestData, robotarmServiceData * resData_ptr, int * errCode)
{
    bool msgLock = true;
    bool dataValid = false;
    MSGReqRegister & msg = registerSendMsg(cmdId, reqDataType, requestData, &dataValid, resData_ptr, errCode, &msgLock);


    //Wait for response.
    if(!deadLine(msgLock)){
        std::lock_guard<std::mutex> lock(mutexForRegMsg);
        msg.isWait = false;
        return false;
    }

    return dataValid;
}
ClientThreeNew::MSGReqRegister & ClientThreeNew::registerSendMsg(RobotArmCMD cmdId, DataTypeEnum reqDataType, const void *requestData, bool * dataValid_ptr, robotarmServiceData * resData_ptr, int * errCode_ptr, bool * msgLock_ptr)
{
    std::lock_guard<std::mutex> lock(mutexForRegMsg);

    //Encode request data.
    char writeBuff[1024];memset(writeBuff, 0, 1024);
    int writeBuffSize = 0;
    dataEncode(cmdId, msgSendCount, reqDataType, requestData, writeBuff, writeBuffSize);
    /**
     * @brief 发送消息并将其压入队列。
     */
    MSGReqRegister newMsg{msgSendCount++, cmdId, dataValid_ptr, true, resData_ptr, errCode_ptr, msgLock_ptr};
    msgQueue.push(newMsg);
    send(writeBuff, writeBuffSize);

    if(debugPrint){
        if(log_ptr){
            log_ptr->info("========================================================");
            log_ptr->info("客户端请求编号[{}], 指令[{}], 封装数据大小[{}]。", msgSendCount - 1, cmdId, writeBuffSize);
        }
    }

    return msgQueue.back();
}
void ClientThreeNew::dataEncode(int cmdId, uint32_t index, DataTypeEnum reqDataType, const void *reqDataNoType, char *sendBuff, int &sendbuffSize)
{
    //Make service header and data.
    robotarmServiceHeader reqHeader;
    robotarmServiceData   reqData;
    memset(reqHeader.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);
    memset(reqData.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);
    sendbuffSize = 0;
    strcpy(reqHeader.val.protocolVer, bProtocolVersion.c_str());
    strcpy(reqHeader.val.armType, armType.c_str());
    reqHeader.val.cmdId = cmdId;
    reqHeader.val.index = index;
    reqHeader.val.errCode = 0;
    switch (reqDataType) {
    case DATATYPE_ENUM_BOOL:{
        reqHeader.val.dataSize = sizeof (bool);
        reqData.boolval = *((bool *)reqDataNoType);
        //spdlog::info("封装bool数据：{}.", reqData.boolval);
        break;
    }
    case DATATYPE_ENUM_CHAR:{
        reqHeader.val.dataSize = sizeof (char);
        reqData.charval = *((char *)reqDataNoType);
        break;
    }
    case DATATYPE_ENUM_INT:{
        reqHeader.val.dataSize = sizeof (int);
        reqData.intval = *((int *)reqDataNoType);
        break;
    }
    case DATATYPE_ENUM_INT3:{
        reqHeader.val.dataSize = sizeof(int) * 3;
        int * degP = (int *)reqDataNoType;
        for(int i = 0; i < 3; ++i)
            reqData.int3[i] = degP[i];
        break;
    }
    case DATATYPE_ENUM_INT6:{
        reqHeader.val.dataSize = sizeof(int) * 6;
        int * degP = (int *)reqDataNoType;
        for(int i = 0; i < 6; ++i)
            reqData.int6[i] = degP[i];
        break;
    }
    case DATATYPE_ENUM_DOUBLE:{
        reqHeader.val.dataSize = sizeof(double);
        reqData.doubleval = *((double *)reqDataNoType);
        break;
    }
    case DATATYPE_ENUM_DOUBLE2:{
        reqHeader.val.dataSize = sizeof(double) * 2;
        double * degP = (double *)reqDataNoType;
        for(int i = 0; i < 2; ++i)
            reqData.double2[i] = degP[i];
        break;
    }
    case DATATYPE_ENUM_DOUBLE6:{
        reqHeader.val.dataSize = sizeof(double) * 6;
        double * degP = (double *)reqDataNoType;
        for(int i = 0; i < 6; ++i)
            reqData.double6[i] = degP[i];
        break;
    }
    case DATATYPE_ENUM_DOUBLE7:{
        reqHeader.val.dataSize = sizeof(double) * 7;
        double * degP = (double *)reqDataNoType;
        for(int i = 0; i < 7; ++i)
            reqData.double7[i] = degP[i];
        break;
    }
    case DATATYPE_ENUM_DOUBLE8:{
        reqHeader.val.dataSize = sizeof(double) * 8;
        double * degP = (double *)reqDataNoType;
        for(int i = 0; i < 8; ++i)
            reqData.double8[i] = degP[i];
        break;
    }
    case DATATYPE_ENUM_CHARARRAY20:{
        char * charP = (char *)reqDataNoType;
        reqHeader.val.dataSize = strlen(charP);
        strcpy(reqData.charArray20, charP);
        break;
    }
    case DATATYPE_ENUM_CHARARRAY100:{
        char * charP = (char *)reqDataNoType;
        reqHeader.val.dataSize = strlen(charP);
        strcpy(reqData.charArray100, charP);
        break;
    }
    case DATATYPE_ENUM_WAYPOINT:{
        reqHeader.val.dataSize = SIZE_OF_PROGRAM_WAYPOINT;
        reqData.programWayPoint = *((ProgramWayPoint *)reqDataNoType);
        break;
    }
    case DATATYPE_ENUM_ROBOTTCP:{
        reqHeader.val.dataSize = SIZE_OF_ROBOT_TCP_INFO;
        reqData.robotTCPInfo = *((RobotTCPInfo *)reqDataNoType);
        break;
    }
    default:
        reqHeader.val.dataSize = 0;
        break;
    }

    //Copy header and data.
    memcpy(sendBuff, reqHeader.byte, SIZE_ROBOTARM_SERVICE_HEADER_COMMAND);
    memcpy(sendBuff + SIZE_ROBOTARM_SERVICE_HEADER_COMMAND, reqData.byte, reqHeader.val.dataSize);
    sendbuffSize = SIZE_ROBOTARM_SERVICE_HEADER_COMMAND + reqHeader.val.dataSize;
}
bool ClientThreeNew::send(const std::string & msg)
{
    base.Write(msg);
    return true;
}
bool ClientThreeNew::send(char * msg, int msgSize)
{
    base.Write(msg, msgSize);
    return true;
}
bool ClientThreeNew::deadLine(const bool & lock, double limit)
{
    double currTime = 0;
    while(lock){
        if(limit <= currTime){
            spdlog::info("[{} : {}] 客户端请求指令超时: {}s。", __FILE__, __LINE__, limit);
            return false;
        }
        currTime += 0.001;
        usleep(1000);
    }

    return true;
}



/**
 * @brief 接受数据进行处理的函数
 * @return
 */
void ClientThreeNew::protocolProcess(const std::string& message)
{
    if(debugPrint){
        if(log_ptr){
            log_ptr->info("服务器反馈消息大小[{}]。", message.size());
        }
    }

    if(msgQueue.empty()){
        if(log_ptr){
            spdlog::warn("在没有新请求时收到服务器消息。");
        }
        return;
    }

    std::lock_guard<std::mutex> lock(mutexForRegMsg);

    MSGReqRegister reqMsg = msgQueue.front();
    msgQueue.pop();
    while (!reqMsg.isWait) {
        spdlog::warn("队列中请求消息被放弃，报文头CMD：{}, Index: {}。尝试处理下一个消息。", reqMsg.reqCMD, reqMsg.msgIndex);
        if(!msgQueue.empty()){
            reqMsg = msgQueue.front();
            msgQueue.pop();
        }
        else{
            robotarmServiceHeader resHeader;
            if(dataDecodeAboutHeader(message, reqMsg, resHeader)){
                spdlog::warn("队列中已经没有等待的请求消息，收到服务器的反馈。报文头CMD：{}, Index: {}。", static_cast<int>(resHeader.val.cmdId), static_cast<uint32_t>(resHeader.val.index));
            }
            return;
        }
    }

    //Decode response data.
    if(!dataDecode(message, reqMsg)){
        *(reqMsg.dataValid_ptr) = false;
        *(reqMsg.lockMsg_ptr) = false;
        return;
    }

    //Process different business.
    if(debugPrint){
        responsePrint(reqMsg);
    }

    *(reqMsg.dataValid_ptr) = true;
    *(reqMsg.lockMsg_ptr) = false;
}
bool ClientThreeNew::dataDecode(const std::string& message, MSGReqRegister & reqMsg)
{
    //Decode header.
    robotarmServiceHeader resHeader;
    if(!dataDecodeAboutHeader(message, reqMsg, resHeader)){
        return false;
    }

    memcpy(reqMsg.resData_ptr->byte, message.c_str() + SIZE_ROBOTARM_SERVICE_HEADER_COMMAND, static_cast<int>(resHeader.val.dataSize));
    return true;
}
bool ClientThreeNew::dataDecodeAboutHeader(const std::string& message, MSGReqRegister & reqMsg, robotarmServiceHeader &resHeader)
{
    memset(resHeader.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);
    memcpy(resHeader.byte, message.c_str(), SIZE_ROBOTARM_SERVICE_HEADER_COMMAND);

    if(strcmp(resHeader.val.protocolVer, bProtocolVersion.c_str())){
        if(log_ptr){
            log_ptr->warn("客户端30001协议版本解析错误。Server: {}, Client: {}。", resHeader.val.protocolVer, bProtocolVersion);
        }
        return false;
    }

    if(strcmp(resHeader.val.armType, armType.c_str())){
        if(log_ptr){
            log_ptr->warn("客户端30001机械臂类型解析错误。Server: {}, Client: {}。", resHeader.val.armType, armType);
        }
        return false;
    }

    //Check cmd.
    if(reqMsg.reqCMD != resHeader.val.cmdId){
        if(log_ptr){
            log_ptr->warn("客户端30001协议指令解析错误: {}。请求指令为: {}。", static_cast<int>(resHeader.val.cmdId), reqMsg.reqCMD);
        }
        return false;
    }


    //Check cmd.
    if(reqMsg.msgIndex != resHeader.val.index){
        if(log_ptr){
            log_ptr->warn("客户端30001协议序号解析错误: {}。请求序号为: {}。", static_cast<uint32_t>(resHeader.val.index), reqMsg.msgIndex);
        }
        return false;
    }

    *(reqMsg.errCode_ptr) = resHeader.val.errCode;

    //get data size.
    int resDataSize = resHeader.val.dataSize;
    if (resDataSize < 0 || (reqMsg.reqCMD != CMD_WAYPOINT_UPDATE && resDataSize > SIZE_OF_ROBOTARM_SERVICE_MAX)){
        if(log_ptr){
            log_ptr->warn("客户端30001协议数据大小解析错误: {}。", resDataSize);
        }
        return false;
    }

    return true;
}
void ClientThreeNew::responsePrint(const MSGReqRegister & reqMsg)
{
    if(boolDataTypeSet.find(reqMsg.reqCMD) != boolDataTypeSet.end()){
        resBoolPrint(reqMsg);
    }
    else if(intDataTypeSet.find(reqMsg.reqCMD) != intDataTypeSet.end()){
        resIntArrayPrint(reqMsg, 1);
    }
    else if(double6DataTypeSet.find(reqMsg.reqCMD) != double6DataTypeSet.end()){
        resDoubleArrayPrint(reqMsg, 6);
    }
    else{
        if(log_ptr){
            log_ptr->info("Response Index[{}], CMD[{}], Cannot process Data type.", reqMsg.msgIndex, reqMsg.reqCMD);
        }
    }
}
void ClientThreeNew::resBoolPrint(const MSGReqRegister & reqMsg)
{
    if(log_ptr){
        log_ptr->info("服务器反馈 Index[{}], CMD[{}], ErrCode[{}], Data {}.", reqMsg.msgIndex, reqMsg.reqCMD,  *(reqMsg.errCode_ptr), (reqMsg.resData_ptr->boolval ? "True" : "False"));
    }
}
void ClientThreeNew::resIntArrayPrint(const MSGReqRegister & reqMsg, int num)
{
    if(log_ptr){
        std::stringstream os;
        os << "服务器反馈 Index[" << reqMsg.msgIndex << "], CMD[" << reqMsg.reqCMD << "], ErrCode[" << *(reqMsg.errCode_ptr) <<  "], Data ";
        int * data_ptr = static_cast<int *>(reqMsg.resData_ptr->int6);
        for(int i = 0; i < num; ++i){
            os << " " << *(data_ptr + i);
        }
        log_ptr->info("{}", os.str());
    }
}
void ClientThreeNew::resDoubleArrayPrint(const MSGReqRegister & reqMsg, int num)
{
    if(log_ptr){
        std::stringstream os;
        os << "服务器反馈 Index[" << reqMsg.msgIndex << "], CMD[" << reqMsg.reqCMD << "], ErrCode[" << *(reqMsg.errCode_ptr) <<  "], Data ";
        double * data_ptr = static_cast<double *>(reqMsg.resData_ptr->double6);
        for(int i = 0; i < num; ++i){
            os << " " << *(data_ptr + i);
        }
        log_ptr->info("{}", os.str());
    }
}




}


//=======================================================================================================================

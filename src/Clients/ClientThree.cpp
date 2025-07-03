#include <iostream>
#include <cstring>
#include "ClientThree.h"
#include "ClientThreeProtocol.h"
#include "Utiles.h"
#include "arcLog.h"

using namespace std;

namespace AeroRobot {

ClientThree::ClientThree(const char * IP, int port, bool newDebugPrint, bool clientBlock) :
    TCPClientBase(IP, port, newDebugPrint, clientBlock), lockForServerRes(false)
{
    memset(resData.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);
}



//Emergency.
bool ClientThree::setEmergenyOn()
{
    bool req = true;
    bool ret2 = requestServer(CMD_EMERGENCY_STOP, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
bool ClientThree::setEmergencyOff()
{
    bool req = false;
    bool ret2 = requestServer(CMD_EMERGENCY_STOP, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

//Set servo state.
bool ClientThree::setServeOn()
{
    bool req = true;
    bool ret2 = requestServer(CMD_SET_SERVO, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
bool ClientThree::setServeOff()
{
    bool req = false;
    bool ret2 = requestServer(CMD_SET_SERVO, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientThree::moveToHome(double s)
{
    bool ret2 = requestServer(CMD_MOVE_HOME, DATATYPE_ENUM_DOUBLE, &s);
    log_ptr->info("请求运动到Home点位: {}。", resDataBool);
    return ret2 && resDataBool;
}

bool ClientThree::moveToZero(double s)
{
    bool ret2 = requestServer(CMD_MOVE_ZERO, DATATYPE_ENUM_DOUBLE, &s);
    log_ptr->info("请求运动到Zero点位: {}。", resDataBool);
    return ret2 && resDataBool;
}

bool ClientThree::setNewHomePos()
{
    bool req = true;
    bool ret2 = requestServer(CMD_SET_HOME_POS, DATATYPE_ENUM_BOOL, &req);
    log_ptr->info("请求设置新的Home点位: {}。", resDataBool);
    return ret2 && resDataBool;
}

//Get robotArm status.
bool ClientThree::getCurrentJoints(double * joints)
{
    bool req = true;
    bool ret2 = requestServer(CMD_GET_JOINT_POSITION, DATATYPE_ENUM_BOOL, &req);
    if(ret2){
        memcpy(joints, currentJoints, 6* sizeof(double));
    }
    else{
        memset(joints, 0, 6* sizeof(double));
    }

    return ret2;
}
bool ClientThree::getCurrentCart(double * pose)
{
    bool req = true;
    bool ret2 = requestServer(CMD_GET_CART_POSITION, DATATYPE_ENUM_BOOL, &req);
    if(ret2){
        memcpy(pose, currentCart, 6* sizeof(double));
    }
    else{
        memset(pose, 0, 6* sizeof(double));
    }

    return ret2;
}
bool ClientThree::getCurrentJointsSpeed(double * speed)
{
    bool req = true;
    bool ret2 = requestServer(CMD_GET_JOINT_VELOCITY, DATATYPE_ENUM_BOOL, &req);
    if(ret2){
        memcpy(speed, currentJointsSpeed, 6* sizeof(double));
    }
    else{
        memset(speed, 0, 6* sizeof(double));
    }

    return ret2;
}

bool ClientThree::getCurrentJointAcce(double *acce)
{
    bool req = true;
    bool ret2 = requestServer(CMD_GET_JOINT_ACCEL, DATATYPE_ENUM_BOOL, &req);
    if(ret2){
        memcpy(acce, currentJointAccel, 6* sizeof(double));
    }
    else{
        memset(acce, 0, 6* sizeof(double));
    }

    return ret2;
}

bool ClientThree::getCurrentJointTemperature(double *temper)
{
    bool req = true;
    bool ret2 = requestServer(CMD_GET_TEMPERATURE, DATATYPE_ENUM_BOOL, &req);
    if(ret2){
        memcpy(temper, currentJointTemperature, 6* sizeof(double));
    }
    else{
        memset(temper, 0, 6* sizeof(double));
    }

    return ret2;
}

bool ClientThree::getCurrentAmpere(double * Ampere)
{
    bool req = true;
    bool ret2 = requestServer(CMD_GET_TORQUE, DATATYPE_ENUM_BOOL, &req);
    if(ret2){
        memcpy(Ampere, currentAmpere, 6* sizeof(double));
    }
    else{
        memset(Ampere, 0, 6* sizeof(double));
    }

    return ret2;
}

bool ClientThree::isHome()
{
    bool req = true;
    bool ret2 = requestServer(CMD_IS_HOME, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}


bool ClientThree::getDoubleEncoderDiff(double * diff_ptr)
{
    bool req = true;
    bool ret2 = requestServer(CMD_GET_DOUBLE_ENCODER_DIFFERENCE, DATATYPE_ENUM_BOOL, &req);
    if(ret2){
        memcpy(diff_ptr, doubleEncoderDiff, 6* sizeof(double));
    }
    else{
        memset(diff_ptr, 0, 6 * sizeof(double));
    }

    return ret2;
}



//Set csp mode.
bool ClientThree::set_csp()
{
    bool req = true;
    bool ret2 = requestServer(CMD_SET_CSP_MODE, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
//Get pose control motion state.
bool ClientThree::isMoveFinished()
{
    bool req = true;
    bool ret2 = requestServer(CMD_IS_MOVE_FINISEHD, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
//Joint space pose control.
bool ClientThree::set_joint_move_speed(double s)
{
    bool ret2 = requestServer(CMD_SET_JOINT_MOVE_SPEED, DATATYPE_ENUM_DOUBLE, &s);
    log_ptr->info("请求设置关节空间运动的速度: {}:{}。", s, resDataBool);
    return ret2 && resDataBool;
}
bool ClientThree::moveJointTo(double * tQ)
{
    bool ret2 = requestServer(CMD_JOINT_MOVE_TO, DATATYPE_ENUM_DOUBLE6, tQ);
    return ret2 && resDataBool;
}
//Cartesian space pose control.
bool ClientThree::set_Task_move_speed(const double * s)
{
    bool ret2 = requestServer(CMD_SET_CART_MOVE_SPEED, DATATYPE_ENUM_DOUBLE2, s);
    log_ptr->info("请求设置笛卡尔空间运动的速度: [{}, {}]:{}。", s[0], s[1], resDataBool);
    return ret2 && resDataBool;
}
bool ClientThree::moveTaskTo(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_TO, DATATYPE_ENUM_DOUBLE6, tC);
    return ret2 && resDataBool;
}
//Continues motion.
bool ClientThree::moveTaskToContinues(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_TO_CONTINUOUS, DATATYPE_ENUM_DOUBLE8, tC);
    return ret2 && resDataBool;
}
bool ClientThree::moveJointToContinueswithSpeed(double * tC)
{
    bool ret2 = requestServer(CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED, DATATYPE_ENUM_DOUBLE7, tC);
    return ret2 && resDataBool;
}
bool ClientThree::moveJointToContinueswithDurat(double * tC)
{
    bool ret2 = requestServer(CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT, DATATYPE_ENUM_DOUBLE7, tC);
    return ret2 && resDataBool;
}
bool ClientThree::moveTaskBy(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_BY, DATATYPE_ENUM_DOUBLE6, tC);
    return ret2 && resDataBool;
}
bool ClientThree::moveTaskByTCP(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_BY_TCP, DATATYPE_ENUM_DOUBLE6, tC);
    return ret2 && resDataBool;
}
bool ClientThree::moveTaskToWithSpeed(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_TO_SPEED, DATATYPE_ENUM_DOUBLE8, tC);
    return ret2 && resDataBool;
}
bool ClientThree::moveStop()
{
    bool req = true;
    bool ret2 = requestServer(CMD_STOP, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

//Set csv mode.
bool ClientThree::set_csv()
{
    bool req = true;
    bool ret2 = requestServer(CMD_SET_CSV_MODE, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
//Joint space speed control.
bool ClientThree::moveJointSpeedTo(const double * ts)
{
    bool ret2 = requestServer(CMD_JOINTSPEED_MOVE_TO, DATATYPE_ENUM_DOUBLE6, ts);
    return ret2 && resDataBool;
}


bool ClientThree::getCurrentRobotTCP(std::string & TCPName, std::array<double, 6> & tcpPosEuler)
{
    bool req = true;
    bool ret2 = requestServer(CMD_GET_CURRENT_TCP, DATATYPE_ENUM_BOOL, &req);

    TCPName = std::string(currentRobotTCPInfo.RobotTCPName);
    for(size_t i = 0; i < 6; ++i){
        tcpPosEuler[i] = currentRobotTCPInfo.RobotPosEuler[i];
    }

    return ret2;
}



bool ClientThree::setCurrentRobotTCP(const std::string & TCPName)
{
    if(TCPName.size() > SIZE_OF_CHAR_ARRAY20_MAX){
        spdlog::warn("{}:{} TCP名字大于20Byte，请重新出入。", __FILE__, __LINE__);
        return false;
    }

    bool ret = requestServer(CMD_SET_CURRENT_TCP, DATATYPE_ENUM_CHARARRAY20, TCPName.c_str());
    return ret && resDataBool;
}



bool ClientThree::setNewCurrentRobotTCP(const std::string & TCPName, const std::array<double, 6> & tcpPosEuler)
{
    if(TCPName.size() > SIZE_OF_CHAR_ARRAY20_MAX){
        spdlog::warn("{}:{} TCP名字大于20Byte，请重新出入。", __FILE__, __LINE__);
        return false;
    }

    RobotTCPInfo tmpTCP;
    strcpy(tmpTCP.RobotTCPName, TCPName.c_str());
    for(size_t i = 0; i < 6; ++i){
        tmpTCP.RobotPosEuler[i] = tcpPosEuler[i];
    }

    bool ret = requestServer(CMD_SET_NEW_TO_CURRENT_TCP, DATATYPE_ENUM_ROBOTTCP, &tmpTCP);
    return ret && resDataBool;
}


bool ClientThree::complexWayPointsPushBack(double * tW)
{
    bool ret2 = requestServer(CMD_COMPLEX_WAYPOINTS_PUSH, DATATYPE_ENUM_DOUBLE6, tW);
    return ret2 && resDataBool;
}


bool ClientThree::complexWayPointsClear()
{
    bool req = true;
    bool ret2 = requestServer(CMD_COMPLEX_WAYPOINTS_CLEAR, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}


bool ClientThree::complexWayPointsRun()
{
    bool req = true;
    bool ret2 = requestServer(CMD_COMPLEX_WAYPOINTS_RUN, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientThree::newProg(const std::string fileName, int &error)
{
    if(fileName.size() > SIZE_OF_CHAR_ARRAY100_MAX){
        spdlog::warn("{}:{} 程序名字大于100Byte，请重新出入。", __FILE__, __LINE__);
        return false;
    }

    bool ret2 = requestServer(CMD_PROGRAM_NEW, DATATYPE_ENUM_CHARARRAY100, fileName.c_str());
    return ret2 && resDataBool;
}


bool ClientThree::openProg(const std::string fileName, int &error)
{
    if(fileName.size() > SIZE_OF_CHAR_ARRAY100_MAX){
        spdlog::warn("{}:{} 程序名字大于100Byte，请重新出入。", __FILE__, __LINE__);
        return false;
    }

    bool ret2 = requestServer(CMD_PROGRAM_OPEN, DATATYPE_ENUM_CHARARRAY100, fileName.c_str());
    return ret2 && resDataBool;
}

bool ClientThree::runProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_RUN, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientThree::stopProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_STOP, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientThree::saveProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_SAVE, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientThree::saveExitProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_SAVE_EXIT, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientThree::exitProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_EXIT, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientThree::pushBackPoint(const ProgramWayPoint & w, int &error)
{
    bool ret2 = requestServer(CMD_WAYPOINT_PUSHBACK, DATATYPE_ENUM_WAYPOINT, &w);
    return ret2 && resDataBool;
}

bool ClientThree::insertPoint(const ProgramWayPoint & w, int &error)
{
    bool ret2 = requestServer(CMD_WAYPOINT_INSERT, DATATYPE_ENUM_WAYPOINT, &w);
    return ret2 && resDataBool;
}

bool ClientThree::erasePoint(int index, int &error)
{
    bool ret2 = requestServer(CMD_WAYPOINT_ERASE, DATATYPE_ENUM_INT, &index);
    return ret2 && resDataBool;
}

bool ClientThree::replacePoint(const ProgramWayPoint & w, int &error)
{
    bool ret2 = requestServer(CMD_WAYPOINT_REPLACE, DATATYPE_ENUM_WAYPOINT, &w);
    return ret2 && resDataBool;
}

bool ClientThree::clearAllPoints(int &error)
{
    bool ret = true;
    bool ret2 = requestServer(CMD_WAYPOINT_CLEAR, DATATYPE_ENUM_BOOL, &ret);
    return ret2 && resDataBool;
}

bool ClientThree::updatePoints(int & line, int &error)
{
    bool ret = true;
    bool ret2 = requestServer(CMD_WAYPOINT_UPDATE, DATATYPE_ENUM_BOOL, &ret);
    if(ret2){
        line = progLine;
    }
    else{
        line = -1;
    }

    return ret2;
}

//Process data.
bool ClientThree::protocolProcess()
{
    //Decode response data.
    if(!dataDecode()){
        lockForServerRes = false;
        return false;
    }

    //Process different business.
    if(!businessProcess()){
        lockForServerRes = false;
        return false;
    }

    //Unlock server response and return.
    lockForServerRes = false;
    return true;
}

//Request to server.
bool ClientThree::requestServer(RobotArmCMD cmdId, DataTypeEnum reqDataType, const void *requestData)
{
    //Encode request data.
    char writeBuff[1024];
    memset(writeBuff, 0, 1024);
    int writeBuffSize = 0;
    dataEncode(cmdId, reqDataType, requestData, writeBuff, writeBuffSize);

    //Log cmd and open switch for waiting response.
    reqCMD = cmdId;
    lockForServerRes = true;

    //Send data to server.
    if(!sendMessage(writeBuff, writeBuffSize)){
        return false;
    }

    //Wait for response.
    return deadLine(lockForServerRes);
}

//Encode data.
void ClientThree::dataEncode(int cmdId, DataTypeEnum reqDataType, const void *reqDataNoType, char *sendBuff, int &sendbuffSize)
{
    //Make service header and data.
    robotarmServiceHeader reqHeader;
    robotarmServiceData   reqData;
    memset(reqHeader.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);
    memset(reqData.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);
    sendbuffSize = 0;
    strcpy(reqHeader.val.robotName, "service");
    reqHeader.val.cmdId = cmdId;
    switch (reqDataType) {
    case DATATYPE_ENUM_BOOL:{
        reqHeader.val.dataSize = sizeof (bool);
        reqData.boolval = *((bool *)reqDataNoType);
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

//Decode data.
bool ClientThree::dataDecode()
{
    //Decode header.
    int resDataSize = 0;
    if(!dataDecodeAboutHeader(resDataSize)){
        return false;
    }

    //Get data from server.
    char readBuff[resDataSize + 10];
    memset(readBuff, 0, resDataSize + 10);
    if(!recvMessageSizeTimeout(readBuff, resDataSize)){
        return false;
    }

    //TODO： support CMD_WAYPOINT_UPDATE.
//    if(reqCMD == CMD_WAYPOINT_UPDATE){
//        errCode = ERR_PROTOCOL_DATA;
//        cout << "We do not support CMD_WAYPOINT_UPDATE now." << endl;
//        return false;
//    }

    //Decode char data to struct.
    memcpy(resData.byte, readBuff, resDataSize);
    return true;
}

//Decode data header.
bool ClientThree::dataDecodeAboutHeader(int &resDataSize)
{
    char readBuff[SIZE_ROBOTARM_SERVICE_HEADER_COMMAND + 10];
    robotarmServiceHeader resHeader;
    memset(readBuff, 0, SIZE_ROBOTARM_SERVICE_HEADER_COMMAND + 10);
    memset(resHeader.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);

    //receive header from service server.
    bool recvState = recvMessageSizeTimeout(readBuff, SIZE_ROBOTARM_SERVICE_HEADER_COMMAND);

    //receive header message error.
    if(!recvState)
        return false;

    //check robotname.
    memcpy(resHeader.byte, readBuff, SIZE_ROBOTARM_SERVICE_HEADER_COMMAND);
    if(strcmp(resHeader.val.robotName, "service")){
        log_ptr->warn("协议头解析错误: {}。", resHeader.val.robotName);
        errCode = ERR_PROTOCOL_WRONG_HEADER;
        return false;
    }

    //Check cmd.
    if(reqCMD != resHeader.val.cmdId){
        log_ptr->warn("协议指令解析错误: {}。请求指令为: {}。", static_cast<int>(resHeader.val.cmdId), reqCMD);
        errCode = ERR_PROTOCOL_WRONG_HEADER;
        return false;
    }

    //get data size.
    resDataSize = resHeader.val.dataSize;
    if (resDataSize < 0 || (reqCMD != CMD_WAYPOINT_UPDATE && resDataSize > SIZE_OF_ROBOTARM_SERVICE_MAX)){
        log_ptr->warn("协议数据大小解析错误: {}。", resDataSize);
        errCode = ERR_PROTOCOL_DATASIZE;
        return false;
    }

    return true;
}

//Process different business.
bool ClientThree::businessProcess()
{
    if(reqCMD == CMD_EMERGENCY_STOP){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_EMERGENCY_STOP", resDataBool);
    }
    else if(reqCMD == CMD_SET_SERVO){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_SERVO", resDataBool);
    }
    else if(reqCMD == CMD_STOP){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_STOP", resDataBool);
    }
    else if(reqCMD == CMD_MOVE_HOME){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_MOVE_HOME", resDataBool);
    }
    else if(reqCMD == CMD_MOVE_ZERO){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_MOVE_ZERO", resDataBool);
    }
    else if(reqCMD == CMD_IS_MOVE_FINISEHD){
        //Execute response from server.
        resDataBool = resData.boolval;
        resBoolWrite(cout, "CMD_IS_MOVE_FINISEHD", resDataBool);
    }
    else if(reqCMD == CMD_IS_HOME){
        //Execute response from server.
        resDataBool = resData.boolval;
        resBoolWrite(cout, "CMD_IS_HOME", resDataBool);
    }
    else if(reqCMD == CMD_JOINT_MOVE_TO){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINT_MOVE_TO", resDataBool);
    }
    else if(reqCMD == CMD_JOINT_MOVE_BY){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINT_MOVE_BY", resDataBool);
    }
    else if(reqCMD == CMD_TASK_MOVE_TO){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_TO", resDataBool);
    }
    else if(reqCMD == CMD_TASK_MOVE_TO_SPEED){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_TO_SPEED", resDataBool);
    }
    else if(reqCMD == CMD_TASK_MOVE_BY){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_BY", resDataBool);
    }
    else if(reqCMD == CMD_TASK_MOVE_BY_TCP){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_BY_TCP", resDataBool);
    }
    else if(reqCMD == CMD_TASK_MOVE_TO_CONTINUOUS){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_TO_CONTINUOUS", resDataBool);
    }
    else if(reqCMD == CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED", resDataBool);
    }
    else if(reqCMD == CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT", resDataBool);
    }
    else if(reqCMD == CMD_SET_JOINT_MOVE_SPEED){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_JOINT_MOVE_SPEED", resDataBool);
    }
    else if(reqCMD == CMD_SET_CART_MOVE_SPEED){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_CART_MOVE_SPEED", resDataBool);
    }
    else if(reqCMD == CMD_SET_CSP_MODE){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_CSP_MODE", resDataBool);
    }
    else if(reqCMD == CMD_SET_CSV_MODE){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_CSV_MODE", resDataBool);
    }
    else if(reqCMD == CMD_JOINTSPEED_MOVE_TO){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINTSPEED_MOVE_TO", resDataBool);
    }
    else if(reqCMD == CMD_GET_JOINT_POSITION){
        //Execute response from server.
        memcpy(currentJoints, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_JOINT_POSITION", currentJoints);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_JOINT_VELOCITY){
        //Execute response from server.
        memcpy(currentJointsSpeed, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_JOINT_VELOCITY", currentJointsSpeed);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_JOINT_ACCEL){
        //Execute response from server.
        memcpy(currentJointAccel, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_JOINT_ACCEL", currentJointAccel);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_TEMPERATURE){
        //Execute response from server.
        memcpy(currentJointTemperature, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_TEMPERATURE", currentJointTemperature);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_TORQUE){
        //Execute response from server.
        memcpy(currentAmpere, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_TORQUE", currentAmpere);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_CART_POSITION){
        //Execute response from server.
        memcpy(currentCart, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_CART_POSITION", currentCart);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_DOUBLE_ENCODER_DIFFERENCE){
        //Execute response from server.
        memcpy(doubleEncoderDiff, resData.double6, 6 * sizeof(double));
//        cout << "CMD_GET_DOUBLE_ENCODER_DIFFERENCE" << " response: ";
//        for(size_t i = 0; i < 6; ++i){
//            cout << " " << doubleEncoderDiff[i];
//        }
//        cout << endl;

        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_CURRENT_TCP){
        //Execute response from server.
        currentRobotTCPInfo = resData.robotTCPInfo;
        resDataBool = true;
    }
    else if(reqCMD == CMD_SET_CURRENT_TCP){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_CURRENT_TCP", resDataBool);
    }
    else if(reqCMD == CMD_SET_NEW_TO_CURRENT_TCP){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_NEW_TO_CURRENT_TCP", resDataBool);
    }
    else if(reqCMD == CMD_PROGRAM_NEW){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_NEW", resDataBool);
    }
    else if(reqCMD == CMD_PROGRAM_OPEN){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_OPEN", resDataBool);
    }
    else if(reqCMD == CMD_PROGRAM_SAVE){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_SAVE", resDataBool);
    }
    else if(reqCMD == CMD_PROGRAM_SAVE_EXIT){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_SAVE_EXIT", resDataBool);
    }
    else if(reqCMD == CMD_PROGRAM_EXIT){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_EXIT", resDataBool);
    }
    else if(reqCMD == CMD_PROGRAM_RUN){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_RUN", resDataBool);
    }
    else if(reqCMD == CMD_PROGRAM_STOP){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_STOP", resDataBool);
    }
    else if(reqCMD == CMD_WAYPOINT_PUSHBACK){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_PUSHBACK", resDataBool);
    }
    else if(reqCMD == CMD_WAYPOINT_INSERT){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_INSERT", resDataBool);
    }
    else if(reqCMD == CMD_WAYPOINT_ERASE){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_ERASE", resDataBool);
    }
    else if(reqCMD == CMD_WAYPOINT_REPLACE){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_REPLACE", resDataBool);
    }
    else if(reqCMD == CMD_WAYPOINT_CLEAR){
        //Execute response from server.
        resDataBool = resData.boolval;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_CLEAR", resDataBool);
    }
    else if(reqCMD == CMD_WAYPOINT_UPDATE){
        progLine = resData.intval;


//        //Execute response from server.
//        if(resDataSize <= 0 || ((resDataSize % SIZE_OF_SERVICE_WAYPOINTTYPE) != 0)){
//            cout << "update error." << endl;
//            return false;
//        }
//        int vectorSize = resDataSize / SIZE_OF_SERVICE_WAYPOINTTYPE;
//        robotarmServiceData resData;
//        for(int i = 0; i < vectorSize; ++i){
//            memcpy(resData.byte, readBuff + i * SIZE_OF_SERVICE_WAYPOINTTYPE, SIZE_OF_SERVICE_WAYPOINTTYPE);
//            cout << "index: " << resData.serviceWaypoint.wayPointIndex
//                 << " PointType: " << resData.serviceWaypoint.wayPointPointType
//                 << " motionType: " << resData.serviceWaypoint.wayPointMotionType
//                 << endl
//                 << "Points: ";
//            for(int j = 0; j < 6; ++j)
//                cout << resData.serviceWaypoint.point[j] << " ";
//            cout << endl;
//        }
    }
    else{
        cout << currentDateTime() << " Recv header: Wrong CMD: " << reqCMD << endl;
        errCode = ERR_PROTOCOL_DATA;
        return false;
    }

    memset(resData.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);

    return true;
}



//Write response result depend on bool resData.
void ClientThree::resBoolWrite(ostream & os, const char * cmd, bool ret)
{
    if(!debugPrint){
        return;
    }

    os << currentDateTime() << " ";
    if(ret){
        os << cmd << " response true." << endl;
    }
    else{
        os << cmd << " response false." << endl;
    }

}

void ClientThree::resDouble6Write(std::ostream & os, const char * cmd, double * double6)
{
    if(!debugPrint){
        return;
    }

    os << currentDateTime() << " ";
    os << cmd << " response: ";
    for(size_t i = 0; i < 6; ++i){
        os << " " << double6[i];
    }
    cout << endl;
}

}

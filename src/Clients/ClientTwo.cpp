#include <iostream>
#include <cstring>
#include "ClientTwo.h"
#include "ClientTwoProtocol.h"
#include "Utiles.h"
#include "arcLog.h"

using namespace std;

namespace AeroRobot {

ClientTwo::ClientTwo(const char * IP, int port, bool newDebugPrint, bool clientBlock) :
    TCPClientBase(IP, port, newDebugPrint, clientBlock), lockForServerRes(false), frameNumber(0)
{
    memset(resData.byte, 0, SIZE_OF_ROBOTARM_SERVICE_MAX);
}



//Emergency.
bool ClientTwo::setEmergenyOn()
{
    bool req = true;
    bool ret2 = requestServer(CMD_EMERGENCY_STOP, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
bool ClientTwo::setEmergencyOff()
{
    bool req = false;
    bool ret2 = requestServer(CMD_EMERGENCY_STOP, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

//Set servo state.
bool ClientTwo::setServeOn()
{
    bool req = true;
    bool ret2 = requestServer(CMD_SET_SERVO, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
bool ClientTwo::setServeOff()
{
    bool req = false;
    bool ret2 = requestServer(CMD_SET_SERVO, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientTwo::moveToHome(double s)
{
    bool ret2 = requestServer(CMD_MOVE_HOME, DATATYPE_ENUM_DOUBLE, &s);
    return ret2 && resDataBool;
}

bool ClientTwo::moveToZero(double s)
{
    bool ret2 = requestServer(CMD_MOVE_ZERO, DATATYPE_ENUM_DOUBLE, &s);
    return ret2 && resDataBool;
}

bool ClientTwo::setNewHomePos()
{
    bool req = true;
    bool ret2 = requestServer(CMD_SET_HOME_POS, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

//Get robotArm status.
bool ClientTwo::getCurrentJoints(double * joints)
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
bool ClientTwo::getCurrentCart(double * pose)
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
bool ClientTwo::getCurrentJointsSpeed(double * speed)
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
bool ClientTwo::getCurrentAmpere(double * Ampere)
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

bool ClientTwo::isHome()
{
    bool req = true;
    bool ret2 = requestServer(CMD_IS_HOME, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}


bool ClientTwo::getDoubleEncoderDiff(double * diff_ptr)
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
bool ClientTwo::set_csp()
{
    bool req = true;
    bool ret2 = requestServer(CMD_SET_CSP_MODE, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
//Get pose control motion state.
bool ClientTwo::isMoveFinished()
{
    bool req = true;
    bool ret2 = requestServer(CMD_IS_MOVE_FINISEHD, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
//Joint space pose control.
bool ClientTwo::set_joint_move_speed(double s)
{
    bool ret2 = requestServer(CMD_SET_JOINT_MOVE_SPEED, DATATYPE_ENUM_DOUBLE, &s);
    return ret2 && resDataBool;
}
bool ClientTwo::moveJointTo(double * tQ)
{
    bool ret2 = requestServer(CMD_JOINT_MOVE_TO, DATATYPE_ENUM_DOUBLE6, tQ);
    return ret2 && resDataBool;
}
//Cartesian space pose control.
bool ClientTwo::set_Task_move_speed(const double * s)
{
    bool ret2 = requestServer(CMD_SET_CART_MOVE_SPEED, DATATYPE_ENUM_DOUBLE2, s);
    return ret2 && resDataBool;
}
bool ClientTwo::moveTaskTo(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_TO, DATATYPE_ENUM_DOUBLE6, tC);
    return ret2 && resDataBool;
}
//Continues motion.
bool ClientTwo::moveTaskToContinues(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_TO_CONTINUOUS, DATATYPE_ENUM_DOUBLE8, tC);
    return ret2 && resDataBool;
}
bool ClientTwo::moveJointToContinueswithSpeed(double * tC)
{
    bool ret2 = requestServer(CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED, DATATYPE_ENUM_DOUBLE7, tC);
    return ret2 && resDataBool;
}
bool ClientTwo::moveJointToContinueswithDurat(double * tC)
{
    bool ret2 = requestServer(CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT, DATATYPE_ENUM_DOUBLE7, tC);
    return ret2 && resDataBool;
}
bool ClientTwo::moveTaskBy(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_BY, DATATYPE_ENUM_DOUBLE6, tC);
    return ret2 && resDataBool;
}
bool ClientTwo::moveTaskByTCP(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_BY_TCP, DATATYPE_ENUM_DOUBLE6, tC);
    return ret2 && resDataBool;
}
bool ClientTwo::moveTaskToWithSpeed(double * tC)
{
    bool ret2 = requestServer(CMD_TASK_MOVE_TO_SPEED, DATATYPE_ENUM_DOUBLE8, tC);
    return ret2 && resDataBool;
}
bool ClientTwo::moveStop()
{
    bool req = true;
    bool ret2 = requestServer(CMD_STOP, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

//Set csv mode.
bool ClientTwo::set_csv()
{
    bool req = true;
    bool ret2 = requestServer(CMD_SET_CSV_MODE, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}
//Joint space speed control.
bool ClientTwo::moveJointSpeedTo(const double * ts)
{
    bool ret2 = requestServer(CMD_JOINTSPEED_MOVE_TO, DATATYPE_ENUM_DOUBLE6, ts);
    return ret2 && resDataBool;
}


bool ClientTwo::complexWayPointsPushBack(double * tW)
{
    bool ret2 = requestServer(CMD_COMPLEX_WAYPOINTS_PUSH, DATATYPE_ENUM_DOUBLE6, tW);
    return ret2 && resDataBool;
}


bool ClientTwo::complexWayPointsClear()
{
    bool req = true;
    bool ret2 = requestServer(CMD_COMPLEX_WAYPOINTS_CLEAR, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}


bool ClientTwo::complexWayPointsRun()
{
    bool req = true;
    bool ret2 = requestServer(CMD_COMPLEX_WAYPOINTS_RUN, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientTwo::newProg(const std::string fileName, int &error)
{
    bool ret2 = requestServer(CMD_PROGRAM_NEW, DATATYPE_ENUM_CHARNAME, fileName.c_str());
    return ret2 && resDataBool;
}


bool ClientTwo::openProg(const std::string fileName, int &error)
{
    bool ret2 = requestServer(CMD_PROGRAM_OPEN, DATATYPE_ENUM_CHARNAME, fileName.c_str());
    return ret2 && resDataBool;
}

bool ClientTwo::runProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_RUN, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientTwo::stopProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_STOP, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientTwo::saveProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_SAVE, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientTwo::saveExitProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_SAVE_EXIT, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientTwo::exitProg(int &error)
{
    bool req = true;
    bool ret2 = requestServer(CMD_PROGRAM_EXIT, DATATYPE_ENUM_BOOL, &req);
    return ret2 && resDataBool;
}

bool ClientTwo::pushBackPoint(const ProgramWayPoint & w, int &error)
{
    bool ret2 = requestServer(CMD_WAYPOINT_PUSHBACK, DATATYPE_ENUM_WAYPOINT, &w);
    return ret2 && resDataBool;
}

bool ClientTwo::insertPoint(const ProgramWayPoint & w, int &error)
{
    bool ret2 = requestServer(CMD_WAYPOINT_INSERT, DATATYPE_ENUM_WAYPOINT, &w);
    return ret2 && resDataBool;
}

bool ClientTwo::erasePoint(int index, int &error)
{
    bool ret2 = requestServer(CMD_WAYPOINT_ERASE, DATATYPE_ENUM_INT, &index);
    return ret2 && resDataBool;
}

bool ClientTwo::replacePoint(const ProgramWayPoint & w, int &error)
{
    bool ret2 = requestServer(CMD_WAYPOINT_REPLACE, DATATYPE_ENUM_WAYPOINT, &w);
    return ret2 && resDataBool;
}

bool ClientTwo::clearAllPoints(int &error)
{
    bool ret = true;
    bool ret2 = requestServer(CMD_WAYPOINT_CLEAR, DATATYPE_ENUM_BOOL, &ret);
    return ret2 && resDataBool;
}

bool ClientTwo::updatePoints(int & line, int &error)
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
bool ClientTwo::protocolProcess()
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
bool ClientTwo::requestServer(RobotArmCMD cmdId, DataTypeEnum reqDataType, const void *requestData)
{
    //Encode request data.
    char writeBuff[1024];
    memset(writeBuff, 0, sizeof(writeBuff));
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
void ClientTwo::dataEncode(int cmdId, DataTypeEnum reqDataType, const void *reqDataNoType, char *sendBuff, int &sendbuffSize)
{
    //Make service header and data.
    robotarmServiceHeader reqHeader;
    robotarmServiceData   reqData;
    sendbuffSize = 0;
    strcpy(reqHeader.val.robotName, "service2");
    reqHeader.val.cmdId = cmdId;
    reqHeader.val.frameNumber = frameNumber;
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
    case DATATYPE_ENUM_CHARNAME:{
        char * charP = (char *)reqDataNoType;
        reqHeader.val.dataSize = strlen(charP);
        strcpy(reqData.programName, charP);
        break;
    }
    case DATATYPE_ENUM_WAYPOINT:{
        reqHeader.val.dataSize = SIZE_OF_PROGRAM_WAYPOINT;
        reqData.programWayPoint = *((ProgramWayPoint *)reqDataNoType);
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
bool ClientTwo::dataDecode()
{
    //Decode header.
    int resDataSize = 0;
    if(!dataDecodeAboutHeader(resDataSize)){
        return false;
    }

    //Get data from server.
    char readBuff[resDataSize + 10];
    if(!recvMessageSizeTimeout(readBuff, resDataSize)){
        return false;
    }

    //Get error code from server.
    char readErr[10];
    if(!recvMessageSizeTimeout(readErr, 4)){
        return false;
    }

    //TODO： support CMD_WAYPOINT_UPDATE.
//    if(reqCMD == CMD_WAYPOINT_UPDATE){
//        errCode = ERR_PROTOCOL_DATA;
//        cout << "We do not support CMD_WAYPOINT_UPDATE now." << endl;
//        return false;
//    }

    //Decode data char data to struct.
    memcpy(resData.byte, readBuff, resDataSize);

    //Decode error char data to struct.
    memcpy(resErr.byte, readErr, 4);

    return true;
}

//Decode data header.
bool ClientTwo::dataDecodeAboutHeader(int &resDataSize)
{
    char readBuff[SIZE_ROBOTARM_SERVICE_HEADER_COMMAND + 10];
    robotarmServiceHeader resHeader;

    //receive header from service server.
    bool recvState = recvMessageSizeTimeout(readBuff, SIZE_ROBOTARM_SERVICE_HEADER_COMMAND);

    //receive header message error.
    if(!recvState)
        return false;

    //Check frame number.
    if(frameNumber != resHeader.val.frameNumber){
        log_ptr->warn("服务器2：协议帧数响应错误: {}。请求帧数为: {}。", static_cast<int>(resHeader.val.frameNumber), frameNumber);
        errCode = ERR_PROTOCOL_WRONG_HEADER;
        return false;
    }

    frameNumber++;

    //check robotname.
    memcpy(resHeader.byte, readBuff, SIZE_ROBOTARM_SERVICE_HEADER_COMMAND);
    if(strcmp(resHeader.val.robotName, "service2")){
        log_ptr->warn("服务器2：协议头解析错误: {}。", resHeader.val.robotName);
        errCode = ERR_PROTOCOL_WRONG_HEADER;
        return false;
    }

    //Check cmd.
    if(reqCMD != resHeader.val.cmdId){
        log_ptr->warn("服务器2：协议指令解析错误: {}。请求指令为: {}。", static_cast<int>(resHeader.val.cmdId), reqCMD);
        errCode = ERR_PROTOCOL_WRONG_HEADER;
        return false;
    }


    //get data size.
    resDataSize = resHeader.val.dataSize;
    if (resDataSize < 0 || (reqCMD != CMD_WAYPOINT_UPDATE && resDataSize > SIZE_OF_ROBOTARM_SERVICE_MAX)){
        log_ptr->warn("服务器2：协议数据大小解析错误: {}。", resDataSize);
        errCode = ERR_PROTOCOL_DATASIZE;
        return false;
    }

    return true;
}

//Process different business.
bool ClientTwo::businessProcess()
{
    if(reqCMD == CMD_EMERGENCY_STOP){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_EMERGENCY_STOP", resDataBool);
        resErrWrite(cout, "CMD_EMERGENCY_STOP server error code", resErrInt);
    }
    else if(reqCMD == CMD_SET_SERVO){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_SERVO", resDataBool);
        resErrWrite(cout, "CMD_SET_SERVO server error code", resErrInt);
    }
    else if(reqCMD == CMD_STOP){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_STOP", resDataBool);
        resErrWrite(cout, "CMD_STOP server error code", resErrInt);
    }
    else if(reqCMD == CMD_MOVE_HOME){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_MOVE_HOME", resDataBool);
        resErrWrite(cout, "CMD_MOVE_HOME server error code", resErrInt);
    }
    else if(reqCMD == CMD_MOVE_ZERO){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_MOVE_ZERO", resDataBool);
        resErrWrite(cout, "CMD_MOVE_ZERO server error code", resErrInt);
    }
    else if(reqCMD == CMD_IS_MOVE_FINISEHD){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        resBoolWrite(cout, "CMD_IS_MOVE_FINISEHD", resDataBool);
        resErrWrite(cout, "CMD_IS_MOVE_FINISEHD server error code", resErrInt);
    }
    else if(reqCMD == CMD_IS_HOME){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        resBoolWrite(cout, "CMD_IS_HOME", resDataBool);
        resErrWrite(cout, "CMD_IS_HOME server error code", resErrInt);
    }
    else if(reqCMD == CMD_JOINT_MOVE_TO){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINT_MOVE_TO", resDataBool);
        resErrWrite(cout, "CMD_JOINT_MOVE_TO server error code", resErrInt);
    }
    else if(reqCMD == CMD_JOINT_MOVE_BY){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINT_MOVE_BY", resDataBool);
        resErrWrite(cout, "CMD_JOINT_MOVE_BY server error code", resErrInt);
    }
    else if(reqCMD == CMD_TASK_MOVE_TO){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_TO", resDataBool);
        resErrWrite(cout, "CMD_TASK_MOVE_TO server error code", resErrInt);
    }
    else if(reqCMD == CMD_TASK_MOVE_TO_SPEED){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_TO_SPEED", resDataBool);
        resErrWrite(cout, "CMD_TASK_MOVE_TO_SPEED server error code", resErrInt);
    }
    else if(reqCMD == CMD_TASK_MOVE_BY){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_BY", resDataBool);
        resErrWrite(cout, "CMD_TASK_MOVE_BY server error code", resErrInt);
    }
    else if(reqCMD == CMD_TASK_MOVE_BY_TCP){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_BY_TCP", resDataBool);
        resErrWrite(cout, "CMD_TASK_MOVE_BY_TCP server error code", resErrInt);
    }
    else if(reqCMD == CMD_TASK_MOVE_TO_CONTINUOUS){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_TASK_MOVE_TO_CONTINUOUS", resDataBool);
        resErrWrite(cout, "CMD_TASK_MOVE_TO_CONTINUOUS server error code", resErrInt);
    }
    else if(reqCMD == CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED", resDataBool);
        resErrWrite(cout, "CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED server error code", resErrInt);
    }
    else if(reqCMD == CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT", resDataBool);
        resErrWrite(cout, "CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT server error code", resErrInt);
    }
    else if(reqCMD == CMD_SET_JOINT_MOVE_SPEED){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_JOINT_MOVE_SPEED", resDataBool);
        resErrWrite(cout, "CMD_SET_JOINT_MOVE_SPEED server error code", resErrInt);
    }
    else if(reqCMD == CMD_SET_CART_MOVE_SPEED){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_CART_MOVE_SPEED", resDataBool);
        resErrWrite(cout, "CMD_SET_CART_MOVE_SPEED server error code", resErrInt);
    }
    else if(reqCMD == CMD_SET_CSP_MODE){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_CSP_MODE", resDataBool);
        resErrWrite(cout, "CMD_SET_CSP_MODE server error code", resErrInt);
    }
    else if(reqCMD == CMD_SET_CSV_MODE){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_SET_CSV_MODE", resDataBool);
        resErrWrite(cout, "CMD_SET_CSV_MODE server error code", resErrInt);
    }
    else if(reqCMD == CMD_JOINTSPEED_MOVE_TO){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_JOINTSPEED_MOVE_TO", resDataBool);
        resErrWrite(cout, "CMD_JOINTSPEED_MOVE_TO server error code", resErrInt);
    }
    else if(reqCMD == CMD_GET_JOINT_POSITION){
        //Execute response from server.
        resErrInt = resErr.error;
        memcpy(currentJoints, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_JOINT_POSITION", currentJoints);
        resErrWrite(cout, "CMD_GET_JOINT_POSITION server error code", resErrInt);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_JOINT_VELOCITY){
        //Execute response from server.
        resErrInt = resErr.error;
        memcpy(currentJointsSpeed, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_JOINT_VELOCITY", currentJointsSpeed);
        resErrWrite(cout, "CMD_GET_JOINT_VELOCITY server error code", resErrInt);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_TORQUE){
        //Execute response from server.
        resErrInt = resErr.error;
        memcpy(currentAmpere, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_TORQUE", currentAmpere);
        resErrWrite(cout, "CMD_GET_TORQUE server error code", resErrInt);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_CART_POSITION){
        //Execute response from server.
        resErrInt = resErr.error;
        memcpy(currentCart, resData.double6, 6* sizeof(double));
        resDouble6Write(cout, "CMD_GET_CART_POSITION", currentCart);
        resErrWrite(cout, "CMD_GET_CART_POSITION server error code", resErrInt);
        resDataBool = true;
    }
    else if(reqCMD == CMD_GET_DOUBLE_ENCODER_DIFFERENCE){
        //Execute response from server.
        resErrInt = resErr.error;
        memcpy(doubleEncoderDiff, resData.double6, 6 * sizeof(double));
//        cout << "CMD_GET_DOUBLE_ENCODER_DIFFERENCE" << " response: ";
//        for(size_t i = 0; i < 6; ++i){
//            cout << " " << doubleEncoderDiff[i];
//        }
//        cout << endl;
        resErrWrite(cout, "CMD_GET_DOUBLE_ENCODER_DIFFERENCE server error code", resErrInt);
        resDataBool = true;
    }
    else if(reqCMD == CMD_PROGRAM_NEW){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_NEW", resDataBool);
        resErrWrite(cout, "CMD_PROGRAM_NEW server error code", resErrInt);
    }
    else if(reqCMD == CMD_PROGRAM_OPEN){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_OPEN", resDataBool);
        resErrWrite(cout, "CMD_PROGRAM_OPEN server error code", resErrInt);
    }
    else if(reqCMD == CMD_PROGRAM_SAVE){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_SAVE", resDataBool);
        resErrWrite(cout, "CMD_PROGRAM_SAVE server error code", resErrInt);
    }
    else if(reqCMD == CMD_PROGRAM_SAVE_EXIT){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_SAVE_EXIT", resDataBool);
        resErrWrite(cout, "CMD_PROGRAM_SAVE_EXIT server error code", resErrInt);
    }
    else if(reqCMD == CMD_PROGRAM_EXIT){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_EXIT", resDataBool);
        resErrWrite(cout, "CMD_PROGRAM_EXIT server error code", resErrInt);
    }
    else if(reqCMD == CMD_PROGRAM_RUN){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_RUN", resDataBool);
        resErrWrite(cout, "CMD_PROGRAM_RUN server error code", resErrInt);
    }
    else if(reqCMD == CMD_PROGRAM_STOP){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_PROGRAM_STOP", resDataBool);
        resErrWrite(cout, "CMD_PROGRAM_STOP server error code", resErrInt);
    }
    else if(reqCMD == CMD_WAYPOINT_PUSHBACK){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_PUSHBACK", resDataBool);
        resErrWrite(cout, "CMD_WAYPOINT_PUSHBACK server error code", resErrInt);
    }
    else if(reqCMD == CMD_WAYPOINT_INSERT){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_INSERT", resDataBool);
        resErrWrite(cout, "CMD_WAYPOINT_INSERT server error code", resErrInt);
    }
    else if(reqCMD == CMD_WAYPOINT_ERASE){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_ERASE", resDataBool);
        resErrWrite(cout, "CMD_WAYPOINT_ERASE server error code", resErrInt);
    }
    else if(reqCMD == CMD_WAYPOINT_REPLACE){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_REPLACE", resDataBool);
        resErrWrite(cout, "CMD_WAYPOINT_REPLACE server error code", resErrInt);
    }
    else if(reqCMD == CMD_WAYPOINT_CLEAR){
        //Execute response from server.
        resDataBool = resData.boolval;
        resErrInt = resErr.error;
        if(!resDataBool){
            errCode = ERR_PROTOCOL_DATA;
        }
        resBoolWrite(cout, "CMD_WAYPOINT_CLEAR", resDataBool);
        resErrWrite(cout, "CMD_WAYPOINT_CLEAR server error code", resErrInt);
    }
    else if(reqCMD == CMD_WAYPOINT_UPDATE){
        progLine = resData.intval;
        resErrInt = resErr.error;


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
void ClientTwo::resBoolWrite(ostream & os, const char * cmd, bool ret)
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

void ClientTwo::resErrWrite(std::ostream & os, const char * cmd, int errData)
{
    if(!debugPrint){
        return;
    }

    os << currentDateTime() << " ";
    os << cmd << " response: ";
    if(errData == ERR_SUCC){
        os << " ERR_SUCC.";
    }
    else if (errData == ERR_FUCTION_CALL) {
        os << " ERR_FUCTION_CALL.";
    }
    else if (errData == ERR_INVALID_PARAMETER) {
        os << " ERR_INVALID_PARAMETER.";
    }
    else if (errData == ERR_COMMUNICATION) {
        os << " ERR_COMMUNICATION.";
    }
    else if (errData == ERR_KINE_INVERSE) {
        os << " ERR_KINE_INVERSE.";
    }
    else if (errData == ERR_EMERGENCY_PRESSED) {
        os << " ERR_EMERGENCY_PRESSED.";
    }
    else if (errData == ERR_NOT_ENABLED) {
        os << " ERR_NOT_ENABLED.";
    }
    else if (errData == ERR_NOT_OFF_ENABLE) {
        os << " ERR_NOT_OFF_ENABLE.";
    }
    else if (errData == ERR_IS_RUNNING) {
        os << " ERR_IS_RUNNING.";
    }
    else if (errData == ERR_CANNOT_OPEN_FILE) {
        os << " ERR_CANNOT_OPEN_FILE.";
    }
    else if (errData == ERR_MOTION_ABNORMAL) {
        os << " ERR_MOTION_ABNORMAL.";
    }


    cout << endl;
}

void ClientTwo::resDouble6Write(std::ostream & os, const char * cmd, double * double6)
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

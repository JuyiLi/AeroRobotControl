#include "ClientOne.h"
#include "ClientOneProtocol.h"
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include "Utiles.h"
#include "arcLog.h"
#include <algorithm>

using namespace std;

namespace AeroRobot {


ClientOne::ClientOne(const char * IP, int port, bool newDebugPrint, bool clientBlock):
    TCPClientBase(IP, port, newDebugPrint, clientBlock), user_robotState_Ptr(nullptr)
{

}


//Process data.
bool ClientOne::protocolProcess()
{
    //Decode response data.
    if(!dataDecode()){
        return false;
    }

    //Process different business.
    if(!businessProcess()){
        return false;
    }

    return true;
}

//Get robot status.
const RobotStatus & ClientOne::getRobotStatus()
{
    return robotStatus;
}
void ClientOne::writeRobotStatus(std::ostream & os)
{
    os << robotStatus << endl;
}

//Control CMD.
void ClientOne::setEmergencyOn()
{
    sendMessage("emON,", 5);
}
void ClientOne::setEmergencyOff()
{
    sendMessage("emOFF,", 6);
}
void ClientOne::setServoOn()
{
    sendMessage("servoON,", 8);
}
void ClientOne::setServoOff()
{
    sendMessage("servoOFF,", 9);
}
void ClientOne::moveStop()
{
    sendMessage("servoSTOP,", 10);
}
void ClientOne::MoveContinuousJoint(int axis, bool forwardMove)
{
    if(!checkAxisNum(axis)){
        return;
    }

    string c("MoveJ,");
    if(forwardMove){
        c.append(to_string(axis));
    }
    else{
        c.append(to_string(-axis));
    }
    c.append(",");

    //cout << c.c_str() << " " << strlen(c.c_str()) << endl;
    sendMessage(c.c_str(), strlen(c.c_str()));

}
void ClientOne::MoveContinuousCartesianPose(int dir, bool forwardMove, int refCoordinate)
{
    static map<int, string> dire = {{1, "x+"}, {2, "y+"}, {3, "z+"}, {-1, "x-"}, {-2, "y-"}, {-3, "z-"}};

    if(dir < 1 || dir > 3){
        return;
    }

    if(refCoordinate < 0 || refCoordinate > 1){
        return;
    }

    string c("MoveCartesian,");
    if(refCoordinate == 0){
        c.append("baseContinuous,");
        if(forwardMove){
            c.append(dire[dir]);
        }
        else{
            c.append(dire[-dir]);
        }
    }
    else if(refCoordinate == 1){
        c.append("TCPContinuous,");
        if(forwardMove){
            c.append(dire[dir]);
        }
        else{
            c.append(dire[-dir]);
        }
    }
    else{

    }
    c.append(",");

    //cout << c.c_str() << " " << strlen(c.c_str()) << endl;
    sendMessage(c.c_str(), strlen(c.c_str()));

}
void ClientOne::MoveContinuousCartesianRota(int dir, bool forwardMove, int refCoordinate)
{
    static map<int, string> dire = {{1, "rx+"}, {2, "ry+"}, {3, "rz+"}, {-1, "rx-"}, {-2, "ry-"}, {-3, "rz-"}};

    if(dir < 1 || dir > 3){
        return;
    }

    if(refCoordinate < 0 || refCoordinate > 1){
        return;
    }

    string c("MoveCartesian,");
    if(refCoordinate == 0){
        c.append("baseContinuous,");
        if(forwardMove){
            c.append(dire[dir]);
        }
        else{
            c.append(dire[-dir]);
        }
    }
    else if(refCoordinate == 1){
        c.append("TCPContinuous,");
        if(forwardMove){
            c.append(dire[dir]);
        }
        else{
            c.append(dire[-dir]);
        }
    }
    else{

    }
    c.append(",");

    //cout << c.c_str() << " " << strlen(c.c_str()) << endl;
    sendMessage(c.c_str(), strlen(c.c_str()));
}
void ClientOne::moveInchingPos(int refCoordinate, int dir, bool forwardMove)
{
    if(dir < 0 || dir > 2){
        return;
    }

    if(refCoordinate < 0 || refCoordinate > 1){
        return;
    }

    string c("MoveInchingPos,");
    c.append(to_string(refCoordinate)+","+to_string(dir)+",");
    if(forwardMove)
    {
        c.append("1,");
    }
    else
    {
        c.append("0,");
    }
    //cout << c.c_str() << " " << strlen(c.c_str()) << endl;
    sendMessage(c.c_str(), strlen(c.c_str()));
}
void ClientOne::moveInchingRot(int refCoordinate, int dir, bool forwardMove)
{
    if(dir < 0 || dir > 2){
        return;
    }

    if(refCoordinate < 0 || refCoordinate > 1){
        return;
    }

    string c("MoveInchingRot,");
    c.append(to_string(refCoordinate)+","+to_string(dir)+",");
    if(forwardMove)
    {
        c.append("1,");
    }
    else
    {
        c.append("0,");
    }
    //cout << c.c_str() << " " << strlen(c.c_str()) << endl;
    sendMessage(c.c_str(), strlen(c.c_str()));
}
void ClientOne::JogSpeedUp()
{
    sendMessage("speedUp,", 8);
}
void ClientOne::JogSpeedDown()
{
    sendMessage("speedDown,", 10);
}

void ClientOne::changeBroadcastCycTime(int time)
{
    string data = "changeCycTime," + to_string(time) + ",";
    sendMessage(data.c_str(), strlen(data.c_str()));
}

bool ClientOne::getRobotStatusPtrRegister(RobotStatus * userData_ptr)
{
    if(userData_ptr == nullptr){
        cout << currentDateTime() << " 传入的数据指针是无效的。" << endl;
        return false;
    }

    user_robotState_Ptr = userData_ptr;

    return true;
}

void ClientOne::getRobotState(RobotState & robotStateG)
{
    robotStateG = robotState;
}

bool ClientOne::dataDecode()
{
    /**
     * @brief 接受一次数据。
     */
    char readBuff[1024];
    memset(readBuff, 0, 1024);
    if(!recvMessageOnceUntil(readBuff, 1024)){
        return false;
    }


    /**
     * @brief 判断本次数据有效性。
     */
    size_t sizeOfReadBuff = strlen(readBuff);
    if(sizeOfReadBuff == 0){
        return true;
    }


    static const char * headerChar_ptr = "iPad";
    static const string headerString(headerChar_ptr);
    static char cacheBuff[2048];
    vector<string> lines;
    /**
     * @brief 1.处理本次数据没有尾的情况。
     */
    if(strchr(readBuff, '\n') == NULL){
        /**
         * @brief 1.1 本次数据没有头，退出。
         */
        if(sizeOfReadBuff == strcspn(readBuff, headerChar_ptr)){
#ifdef _TESTPRINT
        cout << currentDateTime() << "没有头和尾的情况readBuff: " << readBuff << endl;
#endif
            return true;
        }
        else {
            /**
             * @brief 1.2 有头（未必完整）将不完整的1包数据更新到缓存的开始位置。
             */
            memset(readBuff, 0, 1024);
            memcpy(cacheBuff, readBuff, sizeOfReadBuff);
#ifdef _TESTPRINT
        cout << currentDateTime() << "有头，没有尾的情况readBuff: " << readBuff << endl;
        cout << currentDateTime() << "有头，没有尾的情况cacheBuff: " << cacheBuff << endl;
#endif
            return true;
        }
    }


    /**
     * @brief 2.处理本次数据有尾的情况。
     */
    if(strchr(readBuff, '\n') != NULL){
        /**
         * @brief 2.1 本次数据没有头。将不完整的1包数据追加到缓存上。
         */
        if(sizeOfReadBuff == strcspn(readBuff, headerChar_ptr)){
            memcpy(cacheBuff + strlen(cacheBuff), readBuff, sizeOfReadBuff);
            splitString1(lines, cacheBuff, "\n");
#ifdef _TESTPRINT
        cout << currentDateTime() << "没有头，有尾的情况readBuff: " << readBuff << endl;
        cout << currentDateTime() << "没有头，有尾的情况cacheBuff: " << cacheBuff << endl;
        cout << currentDateTime() << "组成一个完整的数据包： " << endl;
        for(auto w : lines){
            cout << w << endl;
        }
#endif
        }
        else{
            splitString1(lines, readBuff, "\n");
        }
    }



    /**
     * @brief 解析每一包受到的完整数据。
     */
    auto sizeLines = lines.size();
    for(size_t i = 0; i < sizeLines; ++i){
        vector<string> tokens;
        splitString1(tokens, lines[i], ",");
#ifdef _TESTPRINT
        cout << currentDateTime() << "处理数据包: " << i << " " << tokens.size() << endl;
        for(auto w : tokens){
            cout << w << "; ";
        }
        cout << endl;
#endif
        if(tokens[0] != "iPad" || tokens.size() < 37){
            //errCode = ERR_PROTOCOL_DATA;
            if(debugPrint){
                cout << currentDateTime() << " 获取广播数据数量异常: " << tokens.size() << endl;
                for(const auto& w : tokens){
                    cout << w << "; ";
                }
                cout << endl;
                continue;
            }
        }


        if(tokens.size() == 37){
            try {
                robotStatus.axisNum = stoi(tokens[1]);
                stringToDouble6(tokens.cbegin() + 2, robotStatus.currentJoint);
                stringToDouble6(tokens.cbegin() + 2 + 6, robotStatus.currentPoseEuler);
                stringToDouble6(tokens.cbegin() + 2 + 6 + 6, robotStatus.currentPoseEulerTCP);
                robotStatus.JogSpeedJoint = stod(tokens[20]);
                robotStatus.JogSpeedPose = stod(tokens[21]);
                robotStatus.JogSpeedEuler = stod(tokens[22]);
                robotStatus.JogInchingPose = stod(tokens[23]);
                robotStatus.JogInchingEuler = stod(tokens[24]);
                stringToDouble6(tokens.cbegin() + 25, robotStatus.currentSpeedJoint);
                stringToDouble6(tokens.cbegin() + 25 + 6, robotStatus.currentAmpereJoint);
            }
            catch(const std::invalid_argument& e ) {
                if(debugPrint){
                    log_ptr->warn("{}:{} 客户端1发现string的无效转化： {}.", __FILE__, __LINE__,  e.what());
                }
            }
            catch(const std::out_of_range & e) {
                if(debugPrint){
                    log_ptr->warn("{}:{} 客户端1发现string的越界转化： {}.", __FILE__, __LINE__,  e.what());
                }
            }
        }
        else if(tokens.size() == 41){
            try {
                robotStatus.axisNum = stoi(tokens[1]);
                stringToDouble6(tokens.cbegin() + 2, robotStatus.currentJoint);
                stringToDouble6(tokens.cbegin() + 2 + 6, robotStatus.currentPoseEuler);
                stringToDouble6(tokens.cbegin() + 2 + 6 + 6, robotStatus.currentPoseEulerTCP);
                robotStatus.JogSpeedJoint = stod(tokens[20]);
                robotStatus.JogSpeedPose = stod(tokens[21]);
                robotStatus.JogSpeedEuler = stod(tokens[22]);
                robotStatus.JogInchingPose = stod(tokens[23]);
                robotStatus.JogInchingEuler = stod(tokens[24]);
                stringToDouble6(tokens.cbegin() + 25, robotStatus.currentSpeedJoint);
                stringToDouble6(tokens.cbegin() + 25 + 6, robotStatus.currentAmpereJoint);
                robotState.isEmergency = stod(tokens[37]);
                robotState.isServoOn = stod(tokens[38]);
                robotState.errorJoints = stod(tokens[39]);
                robotState.isRunning = stod(tokens[40]);
            }
            catch(const std::invalid_argument& e ) {
                if(debugPrint){
                    log_ptr->warn("{}:{} 客户端1发现string的无效转化： {}.", __FILE__, __LINE__,  e.what());
                }
            }
            catch(const std::out_of_range & e) {
                if(debugPrint){
                    log_ptr->warn("{}:{} 客户端1发现string的越界转化： {}.", __FILE__, __LINE__,  e.what());
                }
            }
        }
        else{
            if(debugPrint){
                log_ptr->warn("{}:{} 客户端1： 解析错误的数据数量： {}.", __FILE__, __LINE__,  tokens.size());
            }
        }

        //cout << robotStatus << endl;
    }

    return true;
}

bool ClientOne::businessProcess()
{
    if(!user_robotState_Ptr){
        return true;
    }

    /**
     * @brief 将更新的数据赋值给用户注册的内存。
     */
    memcpy(user_robotState_Ptr, &robotStatus, sizeof(RobotStatus));

    return true;
}

bool ClientOne::checkAxisNum(int axis)
{
    if(axis < 1 || axis > 6){
        return false;
    }

    return true;
}
void ClientOne::stringToDouble6(const vector<string>::const_iterator &stringDataIter, double * data)
{
    for(size_t i = 0; i < 6; ++i){
        data[i] = stod(*(stringDataIter + i));
    }
}


}

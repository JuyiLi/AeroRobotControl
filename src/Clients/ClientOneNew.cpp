#include "ClientOneNew.h"
#include <vector>
#include <string>
#include "Utiles.h"

namespace AeroRobot {

//================================================== 初始化函数 ============================================================

ClientOneNew::ClientOneNew(const std::string& serverAddr, const std::string& name) : base(elThread.loop(), serverAddr, name)
{
    base.SetMessageCallback(std::bind(&ClientOneNew::protocolProcess, this, std::placeholders::_1));

}

bool ClientOneNew::connectToServer()
{
    elThread.Start(true);
    base.Connect();
    usleep(100000);

    double s = 0;
    while (s <= 1.0) {
        usleep(100000);
        log_ptr->warn("查询与服务器的链接情况：");
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

void ClientOneNew::Disconnect()
{
    base.Disconnect();
    usleep(1000);
    elThread.Stop(true);
}

bool ClientOneNew::isConnected()
{
    return base.isConnected();
}


/**
 * @brief 接受数据进行处理的函数
 * @return
 */
void ClientOneNew::protocolProcess(const std::string& message)
{

    dataDecode(message);

    if(debugPrint){
        broadcastPrint(message);
    }

}

void ClientOneNew::broadcastPrint(const std::string& message)
{
    spdlog::info("\n{}", "==============================================================================");
    spdlog::info("Raw: {}", message);
    spdlog::info("{}", currentRS);
    spdlog::info("协议错误: {}, 转化失败： {}, 数据粘包： {}，数据缺失： {}。", statPro, statSto, statMore, statLess);
}
bool ClientOneNew::dataDecode(const std::string& message)
{
    std::vector<std::string> lines;
    splitString1(lines, message.c_str(), "\n");

    auto sizeLines = lines.size();
    if(sizeLines > 1){
        ++statMore;
    }
    for(size_t i = 0; i < sizeLines; ++i){
        std::vector<std::string> tokens;
        splitString1(tokens, lines[i], ",");

//        std::cout << currentDateTime() << "处理数据包: " << i << " " << tokens.size() << std::endl;
//        for(auto w : tokens){
//            std::cout << w << "; ";
//        }
//        std::cout << std::endl;


        if(tokens[0] == "Version1.0"){
            if(tokens.size() < 36){
                statLess++;
                break;
            }

            try {
                currentRS.pVersion = tokens[0];
                currentRS.broadTimes = stol(tokens[1]);
                currentRS.armType = tokens[2];

                for(int i = 0; i < 6; ++i){
                    currentRS.JointCurPos[i] = stod(tokens[3 + i]);
                    currentRS.JointCurVel[i] = stod(tokens[3 + 6 + i]);
                    currentRS.JointCurAcc[i] = stod(tokens[3 + 6 + 6 + i]);
                    currentRS.JointCurAmp[i] = stod(tokens[3 + 6 + 6 + 6 + i]);
                    currentRS.CartTCPPosEuler[i] = stod(tokens[3 + 6 + 6 + 6 + 6 + i]);
                }
                currentRS.emergencyState = stoi(tokens[33]);
                currentRS.servoState = stoi(tokens[34]);
                currentRS.runningState = stoi(tokens[35]);
            }
            catch(const std::invalid_argument& e ) {
                if(debugPrint){
                    if(log_ptr){
                        log_ptr->warn("{}:{} 客户端1发现string的无效转化： {}.", __FILE__, __LINE__,  e.what());
                    }
                }
                statSto++;
            }
            catch(const std::out_of_range & e) {
                if(debugPrint){
                    if(log_ptr){
                        log_ptr->warn("{}:{} 客户端1发现string的越界转化： {}.", __FILE__, __LINE__,  e.what());
                    }
                }
                statSto++;
            }
        }
        else if(tokens[0] == "Version1.1"){
            if(tokens.size() < 40){
                statLess++;
                break;
            }

            try {
                currentRS.pVersion = tokens[0];
                currentRS.broadTimes = stol(tokens[1]);
                currentRS.armType = tokens[2];

                for(int i = 0; i < 6; ++i){
                    currentRS.JointCurPos[i] = stod(tokens[3 + i]);
                    currentRS.JointCurVel[i] = stod(tokens[3 + 6 + i]);
                    currentRS.JointCurAcc[i] = stod(tokens[3 + 6 + 6 + i]);
                    currentRS.JointCurAmp[i] = stod(tokens[3 + 6 + 6 + 6 + i]);
                    currentRS.CartTCPPosEuler[i] = stod(tokens[3 + 6 + 6 + 6 + 6 + i]);
                }
                currentRS.JogSpeedLevel = stoi(tokens[33]);
                currentRS.JogSpeed[0] = stod(tokens[34]);
                currentRS.JogSpeed[1] = stod(tokens[35]);
                currentRS.JogSpeed[2] = stod(tokens[36]);
                currentRS.emergencyState = stoi(tokens[37]);
                currentRS.servoState = stoi(tokens[38]);
                currentRS.runningState = stoi(tokens[39]);
            }
            catch(const std::invalid_argument& e ) {
                if(debugPrint){
                    if(log_ptr){
                        log_ptr->warn("{}:{} 客户端1发现string的无效转化： {}.", __FILE__, __LINE__,  e.what());
                    }
                }
                statSto++;
            }
            catch(const std::out_of_range & e) {
                if(debugPrint){
                    if(log_ptr){
                        log_ptr->warn("{}:{} 客户端1发现string的越界转化： {}.", __FILE__, __LINE__,  e.what());
                    }
                }
                statSto++;
            }
        }
        else{
            if(debugPrint){
                if(log_ptr){
                    log_ptr->warn("客户端1协议版本不支持：{}。", tokens[0]);
                }
            }
            statPro++;
        }

        //cout << robotStatus << endl;
    }

    return true;
}


}

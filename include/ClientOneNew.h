#ifndef CLIENTONENEW_H
#define CLIENTONENEW_H

/******************************************************************************
 * @file        ClientOneNew.h
 * @brief       基于evpp建立新的一号客户端。
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2023-02-17
 * @version     v1.0
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2023-02-17 Version: v1.0
 * @par 描述：
 * </table>
 *****************************************************************************/


#include <string>
#include <thread>
#include <queue>
#include <set>
#include "ClientBaseEVPP.h"
#include "ClientOneNewProtocol.h"

namespace AeroRobot {


class ClientOneNew
{
public:

    /**
     * @brief ClientThreeNew 构造函数
     * @param serverAddr 形式："127.0.0.1:8008"
     * @param 客户端名字。
     */
    ClientOneNew(const std::string& serverAddr, const std::string& name);

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


    /**
     * @brief getRobotStatus 获取最新的机器人信息。
     * @return
     */
    const RobotStatus & getRobotStatus(){return currentRS;}


private:

    /**
     * @brief CallBack接受数据进行处理的函数
     * @param message
     */
    void protocolProcess(const std::string& message);
    bool dataDecode(const std::string& message);
    void broadcastPrint(const std::string& message);

private:
    /**
     * @brief elThread evpp客户端。
     */
    evpp::EventLoopThread elThread;
    ClientBaseEVPP base;

    /**
     * @brief mutexForRegMsg 在信息发送和往队列中注册时上锁。
     */
    std::mutex mutexForRegMsg;
    unsigned long msgSendCount = 0;

    /**
     * @brief debugPrint 打印更多的调试信息。
     */
    bool debugPrint = false;

    RobotStatus currentRS;

    long statPro = 0; //不支持的版本协议或者协议解析不正确
    long statSto = 0; //协议中数据在转化时出现问题
    long statMore = 0;//出现多帧数据包粘在一起的现象
    long statLess = 0;//出现一帧数据包不完整的现象


};

}

#endif // CLIENTONENEW_H

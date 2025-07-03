#ifndef CLIENTBASEEVPP_H
#define CLIENTBASEEVPP_H

/******************************************************************************
 * @file        ClientThreeNew.h
 * @brief       基于evpp建立新的三号服务器。
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2023-03-29
 * @version     v1.1
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2023-02-03 Version: v1.0
 * @par 描述：
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-03-29 Version: v1.1
 * @par 描述：对isConnected函数访问无效指针的bug修复。
 * </table>
 *****************************************************************************/

#include <mutex>
#include <iostream>
#include <stdio.h>

#include "evpp/event_loop.h"
#include "evpp/event_loop_thread.h"
#include "evpp/tcp_client.h"
#include "codec.h"
#include "arcLog.h"


namespace AeroRobot {

/**
 * @brief The ClientBaseEVPP class 客户端.
 */
class ClientBaseEVPP{

public:
    typedef std::function<void(const std::string& message)> StringMessageCallback;

    ClientBaseEVPP(evpp::EventLoop* loop, const std::string& serverAddr, const std::string& name)
        : client_(loop, serverAddr, "Client"),
          codec_(std::bind(&ClientBaseEVPP::OnStringMessage, this, std::placeholders::_1, std::placeholders::_2))
    {
        client_.SetConnectionCallback(std::bind(&ClientBaseEVPP::OnConnection, this, std::placeholders::_1));
        client_.SetMessageCallback(std::bind(&LengthHeaderCodec::OnMessage, &codec_, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief Connect 连接服务器。
     */
    void Connect()
    {
        client_.Connect();
    }

    /**
     * @brief Disconnect 断开服务器。
     */
    void Disconnect()
    {
        client_.Disconnect();
    }

    /**
     * @brief isConnected
     * @return 建立建立返回true;否则返回false。
     */
    bool isConnected()
    {
        if(!connection_){
            if(log_ptr){
                log_ptr->info("没有与服务器建立链接: {}。", client_.remote_addr());
            }
            return false;
        }

        return connection_->IsConnected();
    }

    /**
     * @brief Write 写入数据。
     * @param message
     */
    void Write(const evpp::Slice& message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (connection_){
            codec_.Send(connection_, message);
        }
    }

    void Write(char * msg, int msgSize)
    {
        const evpp::Slice message(msg, msgSize);
        std::lock_guard<std::mutex> lock(mutex_);
        if (connection_){
            codec_.Send(connection_, message);
        }
    }

    void SetMessageCallback(const StringMessageCallback& cb) {
        msg_fn_ = cb;
    }
private:
    /**
     * @brief OnConnection 连接服务器成功、失败以及断开后收到消息的回调函数。
     * @param conn TCP连接。
     */
    void OnConnection(const evpp::TCPConnPtr& conn)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (conn->IsConnected()){
            connection_ = conn;
            if(log_ptr){
                log_ptr->info("与服务器建立连接，地址：{}。", conn->AddrToString());
            }
        } else {
            connection_.reset();
            if(log_ptr){
                log_ptr->info("与服务器断开建立连接，地址：{}。", conn->AddrToString());
            }
        }
    }

    /**
     * @brief OnStringMessage 接受到消息后的回调函数。
     * @param message
     */
    void OnStringMessage(const evpp::TCPConnPtr& conn, const std::string& message)
    {
//        fprintf(stdout, "<<< %s\n", message.c_str());
//        fflush(stdout);
        msg_fn_(message);
    }


private:
    /**
     * @brief client_ 客户端。
     */
    evpp::TCPClient client_;

    /**
     * @brief codec_ 最外层的TCP编码解码。
     */
    LengthHeaderCodec codec_;

    /**
     * @brief mutex_ 锁住写操作。
     */
    std::mutex mutex_;

    /**
     * @brief connection_ TCP连接。
     */
    evpp::TCPConnPtr connection_ = nullptr;


    StringMessageCallback msg_fn_;
};

}
#endif // CLIENTBASEEVPP_H

#ifndef CLIENTTHREENEWCODEC_H
#define CLIENTTHREENEWCODEC_H

/******************************************************************************
 * @file        codec.h
 * @brief       新的三号客户端的编码解码类。
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2023-02-03
 * @version     v1.0
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2023-02-03 Version: v1.0
 * @par 描述：
 * </table>
 *****************************************************************************/

#include "evpp/tcp_conn.h"
#include "evpp/buffer.h"

namespace AeroRobot {

class LengthHeaderCodec {
public:
    typedef std::function<void(const evpp::TCPConnPtr&, const std::string& message)> StringMessageCallback;

    explicit LengthHeaderCodec(const StringMessageCallback& cb) : messageCallback_(cb) {}

    void OnMessage(const evpp::TCPConnPtr& conn, evpp::Buffer* buf)
    {
        //std::cout << "Buf Size: " << buf->size() << std::endl;
        while (buf->size() >= kHeaderLen){
            const int32_t len = buf->PeekInt32();
            if (len > 65536 || len < 0){
                LOG_ERROR << "Invalid length " << len;
                conn->Close();
                break;
            }

            if (buf->size() >= len + kHeaderLen) {
                buf->Skip(kHeaderLen);
                std::string message(buf->NextString(len));
                messageCallback_(conn, message);
                break;
            }else{
                break;
            }
        }
    }

    void Send(evpp::TCPConnPtr conn, const evpp::Slice& message)
    {
        evpp::Buffer buf;
        buf.Append(message.data(), message.size());
        buf.PrependInt32(message.size());
        conn->Send(&buf);
    }

private:
    /**
     * @brief messageCallback_ 初始化时存储的消息回调函数。
     */
    StringMessageCallback messageCallback_;

    /**
     * @brief kHeaderLen 用 kHeaderLen Bytes存储整个消息的长度。
     */
    const static size_t kHeaderLen = sizeof(int32_t);
};

}

#endif // CLIENTTHREENEWCODEC_H

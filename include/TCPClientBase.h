#ifndef TCPCLIENTBASE_H
#define TCPCLIENTBASE_H


/******************************************************************************
 * @file        TCPClientBase.h
 * @brief       简要说明
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2022-07-23
 * @version     v1.5
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2021-11-19 Version: v1.3
 * @par 描述： 稳定版本。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-06-08 Version: v1.4
 * @par 描述：在析构函数中加入关闭客户端。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-07-23 Version: v1.5
 * @par 描述：去掉类中的打印变量，加入spdlog。
 * </table>
 *****************************************************************************/



#include <sys/socket.h>
#include <netinet/in.h>
#include <string>
#include <cstring>
#include <pthread.h>

//TCP client error code.
enum TCPClientErrorCode{
    ERR_NONE = 0,
    ERR_CONNECT_SERVER = 1,
    ERR_TASK_CREATE = 2,
    ERR_CLOSED_SERVER = 3,
    ERR_SOCKET_RECV = 4,
    ERR_SOCKET_TIMEOUT = 5,
    ERR_PROTOCOL_WRONG_HEADER = 6,
    ERR_PROTOCOL_DATASIZE = 7,
    ERR_PROTOCOL_DATA = 8,
};


class TCPClientBase
{
public:
    TCPClientBase(const char * IP, int port, bool newDebugPrint = false, bool clientBlock = false);

    //Connect to server.
    bool connectToServer();

    //Close client.
    void closeClient();

    //DeadLine function. Default = 1s.
    bool deadLine(const bool & lock, double limit = 1);

    //Get error code.
    TCPClientErrorCode getError(){return errCode;}

    //Get client FD.
    int getClientFD(){return clientFD;}


    /**
     * @brief isDebugPrint 控制是否打印本客户端的调试信息。
     * @param p true: 打印调试信息；false: 不打印调试信息。
     */
    void isDebugPrint(bool p){debugPrint = p;}

protected:
    //Please return false if something wrong, and client-self will be closed.
    virtual bool protocolProcess() = 0;

    virtual ~TCPClientBase();

    //Receive message once and return immediately. Received message size may less than "size".
    bool recvMessageOnce(char * buffer, size_t size);
    //Keep waiting for the massage to arrive. Received message size may less than "size".
    bool recvMessageOnceUntil( char * buffer, size_t size);
    //Keep waiting for certain number of massage to arrive. If time out, exit abnormally.
    bool recvMessageSizeTimeout(char * buffer, size_t size);
    //Keep waiting for certain number of massage to arrive. No time out.
    bool recvMessageSizeUntil(char * buffer, size_t size);
    //Send message.
    bool sendMessage(char const * buffer, size_t size);


    /**
     * @brief toErrString 将程序运行中的错误转换为std::string。
     * @param error 输入erron
     * @return
     */
    static inline std::string toErrString(int error)
    {
        return std::string(::strerror(error));
    }


private:
    //Select add FD to fd_set.
    void selectAddFDs();
    //Select clear FD from fd_set.
    void selectClearFD(int FD){FD_CLR(FD, &fdArray);}
    //Select deadline config.
    bool selectDeadlineConfig(int second = 0, int usecond = 70000);
    //Select deadline config.
    int selectMonitor();

    //Thread task process.
    bool CreateThread();
    [[noreturn]] static void *TCPClientTask(void *data);

protected:
    int clientFD;                     //Client FD.
    const char * IP;                  //Server IP.
    int port;                         //Server port.
    bool clientBlockSwitch;           //Switch to control client block.

    /**
     * @brief debugPrint 打印更多的调试信息。
     */
    bool debugPrint;

    TCPClientErrorCode errCode;       //Error code.



private:

    fd_set fdArray;                   //Select fd_set.
    struct timeval selectDeadline;    //Select deadline.
    pthread_t taskID;                 //Thread ID.
    bool threadSwitch;                //Switch to open or close thread.
};

//Write TCPClientErrorCode to ostream.
std::ostream & operator<<(std::ostream &os, const TCPClientErrorCode & code);


#endif // TCPCLIENTBASE_H

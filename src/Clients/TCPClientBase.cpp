#include "TCPClientBase.h"
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <iostream>
#include "arcLog.h"

using namespace std;



TCPClientBase::TCPClientBase(const char * newIP, int newPort, bool newDebugPrint, bool newClientBlock) :
    clientFD(-1), IP(newIP), port(newPort), clientBlockSwitch(newClientBlock),
    debugPrint(newDebugPrint), selectDeadline({0, 8000}), threadSwitch(true)
{


}

TCPClientBase:: ~TCPClientBase()
{
    closeClient();
}


//Connect to server.
bool TCPClientBase::connectToServer()
{
    //Server info initial.
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr(IP);

    //Get socket file descriptor.
    clientFD = ::socket(AF_INET, SOCK_STREAM, 0);

    //Connect to server.
    if (connect(clientFD, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0){
        errCode = ERR_CONNECT_SERVER;
        log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 连接服务器失败 error: {}。", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
        return false;
    }

    //Change socket block property.
    if(clientBlockSwitch){ //SET non-blocking sokcet (MSG_DONTWAIT)
        int curFlags = fcntl(clientFD, F_GETFL, 0);
        int fcntlErr = fcntl(clientFD, F_SETFL, curFlags|O_NONBLOCK);
    }
    if(debugPrint){
        log_ptr->info("[SERVER {}:{}] 客户端FD:{} 连接服务器成功。", IP, port, clientFD);
    }

    //Create thread.
    if(!CreateThread()){
        return false;
    }

    return true;
}

//Close client.
void TCPClientBase::closeClient()
{
    ::close(clientFD);
    selectClearFD(clientFD);
    threadSwitch = false;
    log_ptr->info("[SERVER {}:{}] 客户端FD:{} 主动关闭。", IP, port, clientFD);
    clientFD = -1;
}


//Receive message once and return immediately. Received message size may less than "size".
bool TCPClientBase::recvMessageOnce(char * buffer, size_t size)
{
    //Receive data from client.
    int recvLen = ::recv(clientFD, buffer, size, MSG_DONTWAIT);

    //Check receive error from client.
    if (recvLen == 0){
        log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 关闭连接 error: {}。", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
        errCode = ERR_CLOSED_SERVER;
        return false;
    }

    //Check receive error from client.
    if (recvLen == -1){
        switch(errno){
        case EINVAL: case EBADF: case ECONNRESET: case ENXIO: case EPIPE:{
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv critical error: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
            errCode = ERR_SOCKET_RECV;
            return false;
        }
        case EDQUOT:
        case EFBIG: case EIO: case ENETDOWN: case ENETUNREACH: case ENOSPC:{
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv resource failure: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
            errCode = ERR_SOCKET_RECV;
            return false;
        }
        case EINTR:{//interrupt...
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv resource interrupt.", __FILE__, __LINE__, IP, port, clientFD);
        }
        case EAGAIN:{//No data.
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv no data received.", __FILE__, __LINE__, IP, port, clientFD);
        }
        default:{	//else err code
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} receive message failed: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
            errCode = ERR_SOCKET_RECV;
            return false;
        }
        }
    }

    return true;
}

//Keep waiting for the massage to arrive. Received message size may less than "size".
bool TCPClientBase::recvMessageOnceUntil(char * buffer, size_t size)
{
    //Receive data from client.
    int recvLen = ::recv(clientFD, buffer, size, 0);

    //Check receive error from client.
    if (recvLen == 0){
        log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 关闭连接 error: {}。", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
        errCode = ERR_CLOSED_SERVER;
        return false;
    }

    //Check receive error from client.
    if (recvLen == -1){
        switch(errno){
        case EINVAL: case EBADF: case ECONNRESET: case ENXIO: case EPIPE:{
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv critical error: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
            errCode = ERR_SOCKET_RECV;
            return false;
        }
        case EDQUOT:
        case EFBIG: case EIO: case ENETDOWN: case ENETUNREACH: case ENOSPC:{
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv resource failure: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
            errCode = ERR_SOCKET_RECV;
            return false;
        }
        case EINTR:{//interrupt...
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv resource interrupt.", __FILE__, __LINE__, IP, port, clientFD);
        }
        case EAGAIN:{//No data.
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv no data received.", __FILE__, __LINE__, IP, port, clientFD);
        }
        default:{	//else err code
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} receive message failed: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
            errCode = ERR_SOCKET_RECV;
            return false;
        }
        }
    }

    return true;
}

//Keep waiting for certain number of massage to arrive. If time out, exit abnormally.
bool TCPClientBase::recvMessageSizeTimeout(char * buffer, size_t size)
{
    size_t curr = 0;
    double _limitTimeout = 1, _currTimeout = 0;

    while (curr < size){
        if (_currTimeout > _limitTimeout){
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 接受消息超时 {}s。", __FILE__, __LINE__, IP, port, clientFD, _limitTimeout);
            errCode = ERR_SOCKET_TIMEOUT;
            return false;
        }

        int recvLen = ::recv(clientFD, buffer + curr, size - curr, MSG_DONTWAIT/*diff with MSG_WAITALL*/);

        if (recvLen == 0){
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 关闭连接 error: {}。", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
            errCode = ERR_CLOSED_SERVER;
            return false;
        }
        if (recvLen == -1){
            switch(errno){
            case EINVAL: case EBADF: case ECONNRESET: case ENXIO: case EPIPE:{
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv critical error: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                errCode = ERR_SOCKET_RECV;
                return false;
            }
            case EDQUOT:
            case EFBIG: case EIO: case ENETDOWN: case ENETUNREACH: case ENOSPC:{
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv resource failure: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                errCode = ERR_SOCKET_RECV;
                return false;
            }
            case EINTR:{	//interrupt...
                usleep(1e5);	//100ms
                _currTimeout += 0.1;
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv resource interrupt.", __FILE__, __LINE__, IP, port, clientFD);
                continue;
            }
            case EAGAIN:{//temp error
                usleep(1e5);	//100ms
                _currTimeout += 0.1;
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv no data received.", __FILE__, __LINE__, IP, port, clientFD);
                continue;
            }
            default:{	//else err code
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} receive message failed: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                errCode = ERR_SOCKET_RECV;
                return false;
            }
            }
        }

        curr += recvLen;
        usleep(5e3);	//5ms
        _currTimeout += 0.005;
    }

    return true;
}

//Keep waiting for certain number of massage to arrive. No time out.
bool TCPClientBase::recvMessageSizeUntil(char * buffer, size_t size)
{
    size_t curr = 0;

    while (curr < size){
        int recvLen = ::recv(clientFD, buffer + curr, size - curr, 0);

        if (recvLen == 0){
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 关闭连接 error: {}。", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
            errCode = ERR_CLOSED_SERVER;
            return false;
        }
        if (recvLen == -1){
            switch(errno){
            case EINVAL: case EBADF: case ECONNRESET: case ENXIO: case EPIPE:{
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv critical error: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                errCode = ERR_SOCKET_RECV;
                return false;
            }
            case EDQUOT:
            case EFBIG: case EIO: case ENETDOWN: case ENETUNREACH: case ENOSPC:{
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv resource failure: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                errCode = ERR_SOCKET_RECV;
                return false;
            }
            case EINTR:{	//interrupt...
                usleep(1e5);	//100ms
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv resource interrupt.", __FILE__, __LINE__, IP, port, clientFD);
                continue;
            }
            case EAGAIN:{//temp error
                usleep(1e5);	//100ms
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv no data received.", __FILE__, __LINE__, IP, port, clientFD);
                continue;
            }
            default:{	//else err code
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} recv receive message failed: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                errCode = ERR_SOCKET_RECV;
                return false;
            }
            }
        }

        curr += recvLen;
    }

    return true;
}

//Send data with deadline.
bool TCPClientBase::sendMessage(char const * buffer, size_t size)
{
    size_t curr = 0;
    double _limitTimeout = 1, _currTimeout = 0;
    while (curr < size){
        if (_currTimeout > _limitTimeout){
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 发送消息超时 {}s。", __FILE__, __LINE__, IP, port, clientFD, _limitTimeout);
            return false;
        }

        int sendLen = ::send(clientFD, buffer + curr, size - curr, MSG_DONTWAIT);
        if (sendLen == -1){
            switch(errno){
            case EINVAL: case EBADF: case ECONNRESET: case ENXIO: case EPIPE:{
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} send critical error: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                return false;
            }
            case EDQUOT:
            case EFBIG: case EIO: case ENETDOWN: case ENETUNREACH: case ENOSPC:{
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} send resource failure: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                return false;
            }
            case EINTR:{	//interrupt...
                usleep(1e5);	//100ms
                _currTimeout += 0.1;
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} send resource interrupt.", __FILE__, __LINE__, IP, port, clientFD);
                continue;
            }
            case EAGAIN:{	//temp error
                usleep(1e5);	//100ms
                _currTimeout += 0.1;
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} send temp error.", __FILE__, __LINE__, IP, port, clientFD);
                continue;
            }
            default:{
                log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} send message failed: {}.", __FILE__, __LINE__, IP, port, clientFD, toErrString(errno));
                return false;
            }
            }
        }
        else if (sendLen == 0){
            usleep(1e5);	//100ms
            _currTimeout += 0.1;
            continue;
        }

        curr += sendLen;
        usleep(5e3);	//5ms
        _currTimeout += 0.005;
    }

    return true;
}

//DeadLine function. Default = 1s.
bool TCPClientBase::deadLine(const bool & lock, double limit)
{
    double currTime = 0;
    while(lock){
        if(limit <= currTime){
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 客户端请求指令超时:{}s。", __FILE__, __LINE__, IP, port, clientFD, limit);
            return false;
        }
        currTime += 0.001;
        usleep(1000);
    }

    return true;
}


//
//Select process.
//
//Select deadline config.
bool TCPClientBase::selectDeadlineConfig(int second, int usecond)
{
    //Check arguments
    if(second < 0){
        cout << "Wrong second: " << second << endl;
        return false;
    }else if(usecond < 0){
        cout << "Wrong usecond: " << usecond << endl;
        return false;
    }

    //Set select period(time out)
    selectDeadline.tv_sec = second;
    selectDeadline.tv_usec = usecond;

    return true;
}

//Select add FD to fd_set.
void TCPClientBase::selectAddFDs()
{
    //Clear fd_set.
    FD_ZERO(&fdArray);

    //Try to get serverFDs and add to fd_set.
    FD_SET(clientFD, &fdArray);

    //Reset up deadline for one select monitor.
    selectDeadlineConfig(0, 70000);
}

int TCPClientBase::selectMonitor()
{
    return select(clientFD + 1, &fdArray, NULL, NULL, &selectDeadline);
}


[[noreturn]] void * TCPClientBase::TCPClientTask(void *data)
{
    //Data preprocess.
    TCPClientBase * client_ptr = static_cast<TCPClientBase *>(data);

    //Task running.
    while (client_ptr->threadSwitch){
        //Select monitoring socket IOs.
        client_ptr->selectAddFDs();
        int ret = client_ptr->selectMonitor();

        //Check select timeout/error.
        if(ret == -1 || ret == 0){
            continue;
        }

        //Try to confirm if message comes from clientFD with BROADCAST_SERVER_ENUM. The BROADCAST_SERVER_ENUM server adopts broadcast mode.
        if(FD_ISSET(client_ptr->clientFD, &(client_ptr->fdArray))){
            //SocketErrorCode errCode = ERR_NONE;
            bool ret = client_ptr->protocolProcess();
            if(!ret && (client_ptr->errCode != ERR_NONE)){
                log_ptr->info("[SERVER {}:{}] 客户端FD:{} 客户端出现错误{}将主动关闭。", client_ptr->IP, client_ptr->port, client_ptr->clientFD, client_ptr->errCode);
                client_ptr->closeClient();
            }
        }
    }//while(true)

    if(client_ptr->debugPrint){
        log_ptr->info("[SERVER {}:{}] 客户端FD:{} 消息处理线程将要退出。", client_ptr->IP, client_ptr->port, client_ptr->clientFD);
    }

}

bool TCPClientBase::CreateThread()
{
    //Start Task.
    if(pthread_create(&taskID, NULL, TCPClientTask, this) != 0){
        if(debugPrint){
            errCode = ERR_TASK_CREATE;
            log_ptr->warn("[{}:{}] [SERVER {}:{}] 客户端FD:{} 客户端信息处理线程创建失败。", __FILE__, __LINE__, IP, port, clientFD);
        }
        return false;
    }

    if(debugPrint){
        log_ptr->info("[SERVER {}:{}] 客户端FD:{} 客户端信息处理线程创建成功。", IP, port, clientFD);
    }

    usleep(1000);

    return true;
}


ostream & operator<<(ostream &os, const TCPClientErrorCode & code)
{
    switch (code) {
    case ERR_NONE:{
        os << "ERR_NONE";
        break;
    }
    case ERR_CLOSED_SERVER:{
        os << "ERR_CLOSED_SERVER";
        break;
    }
    case ERR_CONNECT_SERVER:{
        os << "ERR_CLOSED_SERVER";
        break;
    }
    case ERR_TASK_CREATE:{
        os << "ERR_CLOSED_SERVER";
        break;
    }
    case ERR_SOCKET_RECV:{
        os << "ERR_SOCKET_RECV";
        break;
    }
    case ERR_SOCKET_TIMEOUT:{
        os << "ERR_SOCKET_TIMEOUT";
        break;
    }
    case ERR_PROTOCOL_WRONG_HEADER:{
        os << "ERR_PROTOCOL_WRONG_HEADER";
        break;
    }
    case ERR_PROTOCOL_DATASIZE:{
        os << "ERR_PROTOCOL_DATASIZE";
        break;
    }
    case ERR_PROTOCOL_DATA:{
        os << "ERR_PROTOCOL_DATA";
        break;
    }
    default:
        os << "ERR_NONE";
    }

    return os;
}


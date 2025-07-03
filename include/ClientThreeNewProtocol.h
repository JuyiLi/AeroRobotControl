#ifndef CLIENTTHREENEWPROTOCOL_H
#define CLIENTTHREENEWPROTOCOL_H

/******************************************************************************
 * @file        ServerThreeNewProtocol.h
 * @brief       新3客户端的协议
 * @details     详细说明
 * @author      ARC<1131970238@qq.com>
 * @date        2023-09-07
 * @version     v1.6
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2023-02-16 Version: v1.0
 * @par 描述： 从ServerThreeProtocol拷贝。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-02-22 Version: v1.1
 * @par 描述： 修改了robotarmServiceHeaderStruct的内容和大小。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-03-02 Version: v1.2
 * @par 描述： 添加了力传感器的支持。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-03-08 Version: v1.3
 * @par 描述： 报文头添加了index
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-04-12 Version: v1.4
 * @par 描述： 加入了获取目标角度和位姿的协议。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-05-23 Version: v1.5
 * @par 描述： 加入了获取emergency,servo,running状态的协议
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-09-07 Version: v1.6
 * @par 描述： 加入了获取前置伺服轴的协议
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-12-19 Version: v1.7
 * @par 描述： 增加获取前置关节位置、速度和力矩的函数。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2024-03-25 Version: v1.8
 * @par 描述： 增加CMD_COMPLEX_WS_IS_FINISHED查询功能。
 * </table>
 *****************************************************************************/

#include <iostream>



/**
 * @brief 服务器协议相关数据默认SIZE。
 */
enum
{
    SIZE_ROBOTARM_SERVICE_HEADER_COMMAND = 56,
    SIZE_OF_ROBOTARM_SERVICE_MAX         = 300,
    SIZE_OF_CHAR_ARRAY100_MAX            = 100,
    SIZE_OF_CHAR_ARRAY20_MAX             = 20,
};


/**
 * @brief The RobotArmCMD enum 传输协议枚举。
 */
enum RobotArmCMD{

    CMD_EMERGENCY_STOP 			= 1,
    CMD_SET_SERVO 				= 2,
    CMD_SET_BRAKE 				= 3,
    CMD_STOP 					= 4,
    CMD_MOVE_HOME				= 5,
    CMD_MOVE_ZERO				= 6,
    CMD_SET_HOME_POS            = 7,

    CMD_JOINT_MOVE_TO			= 11,
    CMD_JOINT_MOVE_BY			= 12,
    CMD_TASK_MOVE_TO			= 13,
    CMD_TASK_MOVE_BY			= 14,
    CMD_TASK_MOVE_TO_SPEED      = 15,
    CMD_SET_JOINT_MOVE_SPEED    = 16,
    CMD_SET_CART_MOVE_SPEED     = 17,

    CMD_TASK_MOVE_BY_TCP        = 19,

    CMD_TASK_MOVE_TO_CONTINUOUS = 23,
    CMD_JOINT_MOVE_TO_CONTINUES_WITH_SPEED = 25,
    CMD_JOINT_MOVE_TO_CONTINUES_WITH_DURAT = 26,

    CMD_JOINT_MOVE_TO_WITH_DEFINED_SPEED   = 27,
    CMD_TASK_MOVE_TO_WITH_DEFINED_SPEED    = 28,

    CMD_JOINTSPEED_MOVE_TO      = 29,

    CMD_SET_CSP_MODE            = 30,
    CMD_SET_CSV_MODE            = 31,

    CMD_IS_ROBOT_RUNNING        = 40,
    CMD_IS_MOVE_FINISEHD        = 41,
    CMD_IS_HOME                 = 42,

    CMD_JOG_MOVE                = 51,
    CMD_JOG_SPEED_SET           = 52,
    CMD_JOG_SPEED_DSET          = 53,

    CMD_SET_NEW_TO_CURRENT_TCP  = 101,
    CMD_SET_CURRENT_TCP         = 102,
    CMD_GET_CURRENT_TCP         = 103,

    CMD_PREJOINTS_MOVE_TO       = 201,
    CMD_PREJOINTS_STOP          = 202,
    CMD_PREJOINTS_SERVO         = 203,
    CMD_PREJOINTS_ISFINISHED    = 204,
    CMD_PREJOINTS_GET_POSITION  = 205,
    CMD_PREJOINTS_GET_VELOCITY  = 206,
    CMD_PREJOINTS_GET_TORQUE    = 207,

    CMD_AFTERJOINTS_MOVE_TO       = 251,
    CMD_AFTERJOINTS_STOP          = 252,
    CMD_AFTERJOINTS_SERVO         = 253,
    CMD_AFTERJOINTS_ISFINISHED    = 254,
    CMD_AFTERJOINTS_GET_POSITION  = 255,
    CMD_AFTERJOINTS_GET_VELOCITY  = 256,
    CMD_AFTERJOINTS_GET_TORQUE    = 257,

    CMD_GET_JOINT_POSITION		= 320,
    CMD_GET_JOINT_VELOCITY		= 321,
    CMD_GET_CART_POSITION		= 322,
    CMD_GET_CART_VELOCITY		= 323,
    CMD_GET_TORQUE				= 324,
    CMD_GET_JOINT_ACCEL         = 325,
    CMD_GET_TEMPERATURE         = 326,
    CMD_GET_DOUBLE_ENCODER_DIFFERENCE  = 330,
    CMD_GET_EMERGENCY_STATE     = 331,
    CMD_GET_SERVO_STATE         = 332,
    CMD_GET_RUNNING_STATE       = 333,
    CMD_GET_CARTPOS_FROM_JOINT  = 340,
    CMD_GET_JOINTPOS_FROM_CART  = 341,

    CMD_PROGRAM_NEW             = 401,
    CMD_PROGRAM_OPEN            = 402,
    CMD_PROGRAM_SAVE            = 403,
    CMD_PROGRAM_SAVE_EXIT       = 404,
    CMD_PROGRAM_EXIT            = 405,
    CMD_PROGRAM_DELETE          = 406,
    CMD_PROGRAM_RUN             = 407,
    CMD_PROGRAM_STOP            = 408,

    CMD_WAYPOINT_PUSHBACK       = 421,
    CMD_WAYPOINT_INSERT         = 422,
    CMD_WAYPOINT_ERASE          = 423,
    CMD_WAYPOINT_REPLACE        = 424,
    CMD_WAYPOINT_CLEAR          = 425,
    CMD_WAYPOINT_UPDATE         = 426,

    CMD_COMPLEX_WAYPOINTS_PUSH  = 501,
    CMD_COMPLEX_WAYPOINTS_CLEAR = 502,
    CMD_COMPLEX_WAYPOINTS_RUN   = 503,
    CMD_COMPLEX_WS_IS_FINISHED  = 504,
    CMD_COMPLEX_LINE_WP_PUSH    = 510,
    CMD_COMPLEX_LINE_WP_CLEAR   = 511,
    CMD_COMPLEX_LINE_WP_PLAN_RUN= 512,
    CMD_COMPLEX_LINE_WP_PLAN    = 513,
    CMD_COMPLEX_LINE_WP_RUN     = 514,

    CMD_CONNECT_TO_SIXFORCE     = 700,
    CMD_GET_SIXFORCE_DATA       = 701,

    CMD_TEST                    = 600,

    CMD_PROTOCOL_SIZE

};


/**
 * @brief The DataTypeEnum enum 编码时需要传入的数据类型的枚举。
 */
enum DataTypeEnum{
    DATATYPE_ENUM_BOOL,
    DATATYPE_ENUM_CHAR,
    DATATYPE_ENUM_INT,
    DATATYPE_ENUM_INT3,
    DATATYPE_ENUM_INT6,
    DATATYPE_ENUM_DOUBLE,
    DATATYPE_ENUM_DOUBLE2,
    DATATYPE_ENUM_DOUBLE6,
    DATATYPE_ENUM_DOUBLE7,
    DATATYPE_ENUM_DOUBLE8,
    DATATYPE_ENUM_CHARARRAY20,
    DATATYPE_ENUM_CHARARRAY100,
    DATATYPE_ENUM_WAYPOINT,
    DATATYPE_ENUM_ROBOTTCP,
    DATATYPE_ENUM_COUNT
};


/**
 * @brief The ProgramWayPoint struct 适用于编程的数据传输结构体。
 */
struct ProgramWayPoint
{
    /**
     * @brief pointSpace 0：关节空间的点。单位是rad。
     *                   1：笛卡尔空间的点。顺序：（x, y, z, rz, ry, rx）。单位是m和rad。
     */
    int pointSpace;


    /**
     * @brief motionType 0：关节空间运动。1：笛卡尔空间直线运动。
     */
    int motionType;


    /**
     * @brief index 用于插入点、替换点时的位置。从0开始算第一个。如果小于0或者大于现在的总点数量不反应。
     */
    int index;


    /**
     * @brief pointData 插入点数据。
     */
    double pointData[6];


    /**
     * @brief speedForAxisMotion 关节空间运动的速度。
     */
    double speedForAxisMotion;


    /**
     * @brief speedForLineMotion 笛卡尔空间运动的速度。
     */
    double speedForLineMotion[2];

}__attribute__((packed));
#define SIZE_OF_PROGRAM_WAYPOINT sizeof(ProgramWayPoint)

inline std::ostream & operator<<(std::ostream & os, const ProgramWayPoint & pp)
{
    os << "PointSpace: " << pp.pointSpace
       << " MotionType: " << pp.motionType
       << " Index: " << pp.index
       << " speedForAxisMotion: " << pp.speedForAxisMotion
       << " speedForLineMotion: " << pp.speedForLineMotion[0] << " " << pp.speedForLineMotion[1] << std::endl;
    os << "Point Data: ";
    for(size_t i = 0; i < 6; ++i){
        os << pp.pointData[i] << " ";
    }
    os << std::endl;

    return os;
}


/**
 * @brief The RobotTCPInfo struct 传输信息。RobotTCPName不超过20 byte。
 */
struct RobotTCPInfo
{
    char RobotTCPName[SIZE_OF_CHAR_ARRAY20_MAX];
    double RobotPosEuler[6];
}__attribute__((packed));
#define SIZE_OF_ROBOT_TCP_INFO sizeof(RobotTCPInfo)


/**
 * @brief 服务器协议的头struct。
 */
struct robotarmServiceHeaderStruct // --> size 56
{
    char protocolVer[20];
    char armType[20];
    int  cmdId;
    uint32_t index;
    int  errCode;
    int  dataSize;
}__attribute__((packed));



/**
 * @brief 服务器协议的头union。
 */
union robotarmServiceHeader
{
    unsigned char byte[SIZE_OF_ROBOTARM_SERVICE_MAX];
    robotarmServiceHeaderStruct val;
}__attribute__((packed));



/**
 * @brief 服务器协议的数据union。
 */
union robotarmServiceData
{
    unsigned char byte[SIZE_OF_ROBOTARM_SERVICE_MAX];
    bool   boolval;
    char   charval;
    int    intval;
    int    int3[3];
    int    int6[6];
    double doubleval;
    double double2[2];
    double double6[6];
    double double7[7];
    double double8[8];
    char charArray20[SIZE_OF_CHAR_ARRAY20_MAX];
    char charArray100[SIZE_OF_CHAR_ARRAY100_MAX];
    ProgramWayPoint programWayPoint;
    RobotTCPInfo robotTCPInfo;
}__attribute__((packed));



#endif // CLIENTTHREENEWPROTOCOL_H

#ifndef CLIENTTHREEPROTOCOL_H
#define CLIENTTHREEPROTOCOL_H


/******************************************************************************
 * @file        ServerThreeProtocol.h
 * @brief       简要说明
 * @details     详细说明
 * @author      ARC<1131970238@qq.com>
 * @date        2022-09-21
 * @version     v1.10
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2022-04-13 Version: v1.4
 * @par 描述： 添加了CMD_SET_HOME_POS指令；修改了点位运动指令从10之后开始。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-04-13 Version: v1.5
 * @par 描述： 添加双编码差值的获取协议。添加了int 6 数组的数据类型，修改了编码时数据类型枚举的名字：DataTypeEnum,
 *            统一了客户端和服务器端的编码类型。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-04-17 Version: v1.6
 * @par 描述： 修改了ProgramWayPoint结构体的组成，适用与ProgramControl的编程方式。加入了ProgramWayPoint的打印方式。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-04-20 Version: v1.7
 * @par 描述： 添加基于TCP的增量运动。
 * </table>
 *
 * * <table>
 * @par 修改日志 Data: 2022-04-21 Version: v1.8
 * @par 描述： 将robotarmServiceData业务数据名称去掉，突出数据的类型，比如double6数组类型。
 * </table>
 *
 * <table>
 * @par 修改日志 Data: 2022-07-20 Version: v1.9
 * @par 描述： 增加了关节加速度接口和温度接口。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-09-21 Version: v1.10
 * @par 描述： 增加了设置TCP的接口。
 * </table>
 *****************************************************************************/

#include <iostream>



/**
 * @brief 服务器协议相关数据默认SIZE。
 */
enum
{
    SIZE_ROBOTARM_SERVICE_HEADER_COMMAND = 28,
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

    CMD_JOINTSPEED_MOVE_TO      = 29,

    CMD_SET_CSP_MODE            = 30,
    CMD_SET_CSV_MODE            = 31,

    CMD_IS_ROBOT_RUNNING        = 40,
    CMD_IS_MOVE_FINISEHD        = 41,
    CMD_IS_HOME                 = 42,

    CMD_SET_NEW_TO_CURRENT_TCP  = 101,
    CMD_SET_CURRENT_TCP         = 102,
    CMD_GET_CURRENT_TCP         = 103,

    CMD_GET_JOINT_POSITION		= 320,
    CMD_GET_JOINT_VELOCITY		= 321,
    CMD_GET_CART_POSITION		= 322,
    CMD_GET_CART_VELOCITY		= 323,
    CMD_GET_TORQUE				= 324,
    CMD_GET_JOINT_ACCEL         = 325,
    CMD_GET_TEMPERATURE         = 326,

    CMD_GET_DOUBLE_ENCODER_DIFFERENCE  = 330,

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

    CMD_PROTOCOL_SIZE

};


/**
 * @brief The DataTypeEnum enum 编码时需要传入的数据类型的枚举。
 */
enum DataTypeEnum{
    DATATYPE_ENUM_BOOL,
    DATATYPE_ENUM_CHAR,
    DATATYPE_ENUM_INT,
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
struct robotarmServiceHeaderStruct // --> size 28
{
    char robotName[20];
    int  cmdId;
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



#endif // CLIENTTHREEPROTOCOL_H

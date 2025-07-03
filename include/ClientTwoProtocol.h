#ifndef CLIENTTWOPROTOCOL_H
#define CLIENTTWOPROTOCOL_H

/******************************************************************************
 * @file        ClientTwoProtocol.h
 * @brief       简要说明
 * @details     详细说明
 * @author      XueChengqian<xcq_ustb@163.com>
 * @date        2022-07-11
 * @version     v1.0
 * @copyright   Copyright By ARC, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2022-07-11 Version: v1.0
 * @par 描述： 用于客户端2的协议。
 * </table>
 *****************************************************************************/

#include <iostream>


namespace AeroRobot {

//service data size
enum
{
    SIZE_ROBOTARM_SERVICE_HEADER_COMMAND = 32,
    SIZE_OF_ROBOTARM_SERVICE_MAX         = 300,
    SIZE_OF_PROGRAM_NAME_MAX             = 100,
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

    CMD_GET_JOINT_POSITION		= 320,
    CMD_GET_JOINT_VELOCITY		= 321,
    CMD_GET_CART_POSITION		= 322,
    CMD_GET_CART_VELOCITY		= 323,
    CMD_GET_TORQUE				= 324,

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
    DATATYPE_ENUM_CHARNAME,
    DATATYPE_ENUM_WAYPOINT,
    DATATYPE_ENUM_COUNT
};


/**
 * @brief 错误码枚举。
 */
enum ErrorCode{
    ERR_SUCC                   = 0,
    ERR_FUCTION_CALL           = 1,
    ERR_INVALID_PARAMETER      = 2,
    ERR_COMMUNICATION          = 3,
    ERR_KINE_INVERSE           = 4,
    ERR_EMERGENCY_PRESSED      = 5,
    ERR_NOT_ENABLED            = 6,
    ERR_NOT_OFF_ENABLE         = 7,
    ERR_IS_RUNNING             = 8,
    ERR_CANNOT_OPEN_FILE       = 9,
    ERR_MOTION_ABNORMAL        = 10
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


//service header struct
struct robotarmServiceHeaderStruct // --> size 32
{
    char robotName[20];
    int  cmdId;
    int  dataSize;
    int  frameNumber;
}__attribute__((packed));


//service header union
union robotarmServiceHeader
{
    char byte[SIZE_OF_ROBOTARM_SERVICE_MAX];
    robotarmServiceHeaderStruct val;
}__attribute__((packed));


//service data union
union robotarmServiceData
{
    char byte[SIZE_OF_ROBOTARM_SERVICE_MAX];
    bool   boolval;
    char   charval;
    int    intval;
    int    int6[6];
    double doubleval;
    double double2[2];
    double double6[6];
    double double7[7];
    double double8[8];
    char programName[SIZE_OF_PROGRAM_NAME_MAX];
    ProgramWayPoint programWayPoint;
}__attribute__((packed));

union robotarmServiceError
{
    char byte[10];
    int  error;
}__attribute__((packed));

}


#endif // CLIENTTWOPROTOCOL_H

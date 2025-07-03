#ifndef ROBOTERRORCODE_H
#define ROBOTERRORCODE_H

/******************************************************************************
 * @file        RobotErrorCode.h
 * @brief       为系统程序提供全局错误码
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2023-02-23
 * @version     v2.1
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2022-10-19 Version: v1.0
 * @par 描述：第一版
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-10-20 Version: v1.1
 * @par 描述： 加入无错误。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-10-20 Version: v1.2
 * @par 描述： 无错误数值修改了0。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-10-20 Version: v1.3
 * @par 描述： 添加ROBOT_EXCEED_VELOCITY_LIMIT
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-10-27 Version: v1.4
 * @par 描述： 添加ROBOT_JOINT_SPACE_SAME_POS等点位数据判断。
 * </table>
 *
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-10-27 Version: v1.4
 * @par 描述： 添加ROBOT_JOINT_SPACE_SAME_POS等点位数据判断。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-10-31 Version: v1.5
 * @par 描述： 添加CONTROL_MODE_UNMATCHED。
 * </table>
 *
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-11-05 Version: v1.6
 * @par 描述：添加ROBOT_INVALID_TRAJACTORY
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-11-10 Version: v1.7
 * @par 描述：增加了各个关节的CSP模式下的计算期望与当前位置超过限制。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-11-18 Version: v1.8
 * @par 描述：增加了ROBOT_PATH_GENERATION_FAILED与运动学相关。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2022-11-23 Version: v1.9
 * @par 描述：增加了ECAT相关。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-02-22 Version: v2.0
 * @par 描述：增添了网络通讯相关错误。
 * </table>
 *
 *
 * <table>
 * @par 修改日志 Data: 2023-02-23 Version: v2.1
 * @par 描述：增添了测试错误。
 * </table>
 *****************************************************************************/

namespace Robot_Param {


// ================================================================= 控制器部分错误代码 ================================================================


/**
 * @brief 通用错误。
 */
#define	ROBOT_NO_ERROR                     0          //没有错误。
#define	ROBOT_POWERED_OFF                  10001      //machine is now off please turn on it 机器人关闭，请开启
#define	ROBOT_NOT_ENABLED                  10002      //machine is not enabled 机器人在下使能状态。
#define	ROBOT_ENABLED                      10003      //machine is still enabled 机器人在上使能状态。
#define	ROBOT_SOFT_EMERGENCY_ENABLED       10004      //machine soft emergency enabled 机器人软急停生效。
#define	ROBOT_SOFT_EMERGENCY_NOT_ENABLED   10005      //machine soft emergency not enabled 机器人软急停失效。
#define	ROBOT_HIT_POS_HARD_LIMIT           10006      //cannot move on since it hits the joint's positive hard limit 机器人关节达到正的硬限位 不可运动
#define	ROBOT_HIT_NEG_HARD_LIMIT           10007      //cannot move on since it hits the joint's negative hard limit 机器人关节达到负的硬限位 不可运动
#define	ROBOT_HIT_POS_SOFT_LIMIT           10008      //cannot move on since it hits the joint's positive soft limit 机器人关节达到正的软限位 不可运动
#define	ROBOT_HIT_NEG_SOFT_LIMIT           10009      //cannot move on since it hits the joint's negative soft limit 机器人关节达到负的软限位 不可运动
#define	ROBOT_INVALID_JOINT_NUM            10010      //the commanded joint number is invalid 指令关节无效
#define	ROBOT_INVALID_VELOCITY             10011      //the commanded velocity is invalid	指令速度无效
#define	ROBOT_KINEMATICS_FAILED            10012      //failed to do the inverse kinematics 运动学逆解失败
#define	ROBOT_STOP_ON_SIGULARITY           10013      //Singularity position protective stop 到达奇异点保护性停止
#define	ROBOT_IS_RUNNING                   10014      //Robot is running. 机器人正在运动。
#define	ROBOT_IS_NOT_RUNNING               10015      //Robot is running. 机器人没有运动。
#define	ROBOT_EXCEED_VELOCITY_LIMIT        10016      //Robot exceed vel limit.	机器人运动速度超过限制
#define	ROBOT_JOINT_SPACE_SAME_POS         10017      //机器人关节空间点位相同
#define	ROBOT_JOINT_SPACE_DIFF_POS         10018      //机器人关节空间点位不同
#define	ROBOT_CART_SPACE_SAME_POS          10019      //机器人笛卡尔空间点位相同
#define	ROBOT_CART_SPACE_DIFF_POS          10020      //机器人笛卡尔空间点位不同
#define ROBOT_TRAJACTORY_GENERATION_FAILED 10021      //机器人轨迹生成失败
#define ROBOT_LEFT_TRAGENERATION_FAILED    10022      //机器人轨迹开始段生成失败
#define ROBOT_MIDDLE_TRAGENERATION_FAILED  10023      //机器人轨迹中间段生成失败
#define ROBOT_RIGHT_TRAGENERATION_FAILED   10024      //机器人轨迹末尾段生成失败
#define	ROBOT_INVALID_MOTION_DIR           10025      //the motion direction is invalid 运动方向无效
#define ROBOT_INVALID_TRAJACTORY           10026      //机器人轨迹数据异常
#define ROBOT_KINEMATICS_DISCOUNTINUOUS    10027      //运动学逆解不连续
#define ROBOT_KINEMATICS_NONUMSOLUTION     10028      //运动学无数值解
#define ROBOT_PATH_GENERATION_FAILED       10029      //机器人路径生成失败



#define ROBOT_SERVO_ERR                    10030      //Robot servo err. 402状态机反馈机器人关节驱动器错误。至少有一个关节驱动器报错。
#define ROBOT_SERVO_ERR_J0                 10031      //Robot servo err. 402状态机反馈机器人关节0驱动器错误。
#define ROBOT_SERVO_ERR_J1                 10032      //Robot servo err. 402状态机反馈机器人关节1驱动器错误。
#define ROBOT_SERVO_ERR_J2                 10033      //Robot servo err. 402状态机反馈机器人关节2驱动器错误。
#define ROBOT_SERVO_ERR_J3                 10034      //Robot servo err. 402状态机反馈机器人关节3驱动器错误。
#define ROBOT_SERVO_ERR_J4                 10035      //Robot servo err. 402状态机反馈机器人关节4驱动器错误。
#define ROBOT_SERVO_ERR_J5                 10036      //Robot servo err. 402状态机反馈机器人关节5驱动器错误。

#define	JOINT_IS_RUNNING_J0                10050      //Robot Joint 0 is running. 机器人关节0正在运动。
#define	JOINT_IS_RUNNING_J1                10051      //Robot Joint 1 is running. 机器人关节1正在运动。
#define	JOINT_IS_RUNNING_J2                10052      //Robot Joint 2 is running. 机器人关节2正在运动。
#define	JOINT_IS_RUNNING_J3                10053      //Robot Joint 3 is running. 机器人关节3正在运动。
#define	JOINT_IS_RUNNING_J4                10054      //Robot Joint 4 is running. 机器人关节4正在运动。
#define	JOINT_IS_RUNNING_J5                10055      //Robot Joint 5 is running. 机器人关节5正在运动。


#define	RT_THREAD_TIME_INVALID            10060      //实时线程时间异常。
#define	CONTROL_MODE_INVALID              10061      //控制模式无效。
#define	CSP_ROBOT_TARCUR_LIMIT            10062      //机器人在CSP模式下的计算期望与当前位置超过限制。
#define	CONTROL_MODE_UNMATCHED            10063      //控制模式不匹配。
#define	ROBOT_EXCEED_TORQUE_LIMIT         10064      //Robot exceed torque limit.	机器人运动力矩超过限制


#define	CSP_TARCUR_LIMIT_J0               10100      //机器人关节0在CSP模式下的计算期望与当前位置超过限制。
#define	CSP_TARCUR_LIMIT_J1               10101      //机器人关节1在CSP模式下的计算期望与当前位置超过限制。
#define	CSP_TARCUR_LIMIT_J2               10102      //机器人关节2在CSP模式下的计算期望与当前位置超过限制。
#define	CSP_TARCUR_LIMIT_J3               10103      //机器人关节3在CSP模式下的计算期望与当前位置超过限制。
#define	CSP_TARCUR_LIMIT_J4               10104      //机器人关节4在CSP模式下的计算期望与当前位置超过限制。
#define	CSP_TARCUR_LIMIT_J5               10105      //机器人关节5在CSP模式下的计算期望与当前位置超过限制。



/**
 * @brief 位置限制 joint pos exceed the limit 关节位置度超过限制
 */
#define JOINT_HIT_POS_SLIMIT_J0 20001  //hits the joint 0's positive soft limit 关节一到达正软限位
#define JOINT_HIT_POS_SLIMIT_J1 20002  //hits the joint 1's positive soft limit 关节二到达正软限位
#define JOINT_HIT_POS_SLIMIT_J2 20003  //hits the joint 2's positive soft limit 关节三到达正软限位
#define JOINT_HIT_POS_SLIMIT_J3 20004  //hits the joint 3's positive soft limit 关节四到达正软限位
#define JOINT_HIT_POS_SLIMIT_J4 20005  //hits the joint 4's positive soft limit 关节五到达正软限位
#define JOINT_HIT_POS_SLIMIT_J5 20006  //hits the joint 5's positive soft limit 关节六到达正软限位
#define JOINT_HIT_NEG_SLIMIT_J0 20007  //hits the joint 0's negative soft limit 关节一到达负软限位
#define JOINT_HIT_NEG_SLIMIT_J1 20008  //hits the joint 1's negative soft limit 关节二到达负软限位
#define JOINT_HIT_NEG_SLIMIT_J2 20009  //hits the joint 2's negative soft limit 关节三到达负软限位
#define JOINT_HIT_NEG_SLIMIT_J3 20010  //hits the joint 3's negative soft limit 关节四到达负软限位
#define JOINT_HIT_NEG_SLIMIT_J4 20011  //hits the joint 4's negative soft limit 关节五到达负软限位
#define JOINT_HIT_NEG_SLIMIT_J5 20012  //hits the joint 5's negative soft limit 关节六到达负软限位

/**
 * @brief 速度限制 joint velocity exceed the limit 关节运动速度超过限制
 */
#define	JOINT_EXCEED_VEL_LIMIT_J0  20021 //关节一运动速度超过限制
#define	JOINT_EXCEED_VEL_LIMIT_J1  20022 //关节二运动速度超过限制
#define	JOINT_EXCEED_VEL_LIMIT_J2  20023 //关节三运动速度超过限制
#define	JOINT_EXCEED_VEL_LIMIT_J3  20024 //关节四运动速度超过限制
#define	JOINT_EXCEED_VEL_LIMIT_J4  20025 //关节五运动速度超过限制
#define	JOINT_EXCEED_VEL_LIMIT_J5  20026 //关节六运动速度超过限制



/**
 * @brief JOG错误：jog joint further past max soft limit JOG目标位置超过关节软限位
 */
#define JOG_TARGET_EXCEED_POS_SLIMIT_J0 30001 //JOG目标位置超过关节一正软限位
#define JOG_TARGET_EXCEED_POS_SLIMIT_J1 30002 //JOG目标位置超过关节二正软限位
#define JOG_TARGET_EXCEED_POS_SLIMIT_J2 30003 //JOG目标位置超过关节三正软限位
#define JOG_TARGET_EXCEED_POS_SLIMIT_J3 30004 //JOG目标位置超过关节四正软限位
#define JOG_TARGET_EXCEED_POS_SLIMIT_J4 30005 //JOG目标位置超过关节五正软限位
#define JOG_TARGET_EXCEED_POS_SLIMIT_J5 30006 //JOG目标位置超过关节六正软限位
#define JOG_TARGET_EXCEED_NEG_SLIMIT_J0 30007 //JOG目标位置超过关节一负软限位
#define JOG_TARGET_EXCEED_NEG_SLIMIT_J1 30008 //JOG目标位置超过关节二负软限位
#define JOG_TARGET_EXCEED_NEG_SLIMIT_J2 30009 //JOG目标位置超过关节三负软限位
#define JOG_TARGET_EXCEED_NEG_SLIMIT_J3 30010 //JOG目标位置超过关节四负软限位
#define JOG_TARGET_EXCEED_NEG_SLIMIT_J4 30011 //JOG目标位置超过关节五负软限位
#define JOG_TARGET_EXCEED_NEG_SLIMIT_J5 30012 //JOG目标位置超过关节六负软限位



/**
 * @brief 碰撞错误：hits the joint positive soft limit 关节到达正软限位
 */
#define	SINGULARITY_PROTECTION            40001   //arrive sigularity position protective stop 到达奇异点保护性停止
#define	COLLISION_PROTECTION              40002   // collision detected pretective stop 检测到碰撞 保护性停止


/**
 * @brief 拖动示教错误
 */
#define	DRAG_CANNOT_ENABLE_ON_LIMIT       40020   //On Soft limit position can not get into drag mode 在软限位上没法进入拖拽模式
#define	CANNOT_DRAG_WHEN_SERVOOFF         40021   //cannot enter drage mode when robot is disabled 机器人未使能无法进入拖拽
#define	CANNOT_DRAG_DURING_MOVEMENT       40022   //cannot enter drage mode when robot is in movement 机器人正在运动无法进入拖拽
#define	CANNOT_SET_COLLISION_SENSITIVITY  40023   //0x0F0055 cannot set collision sensitivity in drag mode or force control mode 拖拽期间不能设置碰撞灵敏度
#define	DRAG_NEARBY_SOFT_LIMIT_J0         40024   //joint 0 is now nearby the soft limit 关节一即将达到软限位
#define	DRAG_NEARBY_SOFT_LIMIT_J1         40025   //joint 1 is now nearby the soft limit 关节二即将达到软限位
#define	DRAG_NEARBY_SOFT_LIMIT_J2         40026   //joint 2 is now nearby the soft limit 关节三即将达到软限位
#define	DRAG_NEARBY_SOFT_LIMIT_J3         40027   //joint 3 is now nearby the soft limit 关节四即将达到软限位
#define	DRAG_NEARBY_SOFT_LIMIT_J4         40028   //joint 4 is now nearby the soft limit 关节五即将达到软限位
#define	DRAG_NEARBY_SOFT_LIMIT_J5         40029   //joint 5 is now nearby the soft limit 关节六即将达到软限位

#define	JOINT_FOLLOWING_ERROR_J0          40030  //joint 0 following error detected  关节一跟随误差过大
#define	JOINT_FOLLOWING_ERROR_J1          40031  //joint 1 following error detected  关节二跟随误差过大
#define	JOINT_FOLLOWING_ERROR_J2          40032  //joint 2 following error detected  关节三跟随误差过大
#define	JOINT_FOLLOWING_ERROR_J3          40033  //joint 3 following error detected  关节四跟随误差过大
#define	JOINT_FOLLOWING_ERROR_J4          40034  //joint 4 following error detected  关节五跟随误差过大
#define	JOINT_FOLLOWING_ERROR_J5          40035  //joint 5 following error detected  关节六跟随误差过大


/**
 * @brief 编程错误。
 */
#define	PROGRAM_INVALID_SYNTAX     50001  //syntax error detected in the program file 编程文件语法错误
#define	PROGRAM_FILE_NOT_OPEN      50002  //no program file is opened 没有编程文件打开
#define	PROGRAM_FILE_FAILED_OPEN   50003  //failed to open the program file	编程文件打开失败
#define	PROGRAM_FILE_FAILED_CLOSE  50004  //failed to close the program file 编程文件关闭失败



/**
 * @brief 网络接口错误。
 */
#define	INTTCP_ERR_PARSE_PROVER      60001  //Error 解析协议版本错误
#define	INTTCP_ERR_PARSE_ARMTYPE     60002  //Error 解析协议机械臂类型错误
#define	INTTCP_ERR_PARSE_DATASIZE    60003  //Error 解析协议数据大小错误
#define	INTTCP_ERR_SYNTAX_FAILED     60004  //Syntax Error 解析失败，语法错误
#define	INTTCP_ERR_SOCKET_CREATE     60005  //Net Connect Error 网络连接失败
#define	INTTCP_ERR_SOCKET_RECV_DATA  60006  //failed to receive data via tcp/ip connection TCP/IP接收数据失败
#define	INTTCP_ERR_SOCKET_SEND_DATA  60007  //failed to send data via tcp/ip connection TCP/IP发送数据失败
#define	INTTCP_ERR_FUCTION_CALL      60008  //调用了未定义的功能
#define	INTTCP_ERR_INVALID_PARAMETER 60009  //参数错误
#define	INTTCP_ERR_CANNOT_OPEN_FILE  60013  //打开文件失败






// ========================================================================================================================================================


// ===================================================================== 伺服部分错误代码 ====================================================================


/**
 * @brief joint phase current over limit 关节输出过流
 */
#define DRIVE_ERR_PHASE_OVERCURRENT_J0 70001
#define DRIVE_ERR_PHASE_OVERCURRENT_J1 70002
#define DRIVE_ERR_PHASE_OVERCURRENT_J2 70003
#define DRIVE_ERR_PHASE_OVERCURRENT_J3 70004
#define DRIVE_ERR_PHASE_OVERCURRENT_J4 70005
#define DRIVE_ERR_PHASE_OVERCURRENT_J5 70006


/**
 * @brief joint servo under voltage 关节伺服欠压
 */
#define DRIVE_ERR_UNDER_VOLTAGE_J0    70011
#define DRIVE_ERR_UNDER_VOLTAGE_J1    70012
#define DRIVE_ERR_UNDER_VOLTAGE_J2    70013
#define DRIVE_ERR_UNDER_VOLTAGE_J3    70014
#define DRIVE_ERR_UNDER_VOLTAGE_J4    70015
#define DRIVE_ERR_UNDER_VOLTAGE_J5    70016


/**
 * @brief joint servo over voltage 关节伺服过压
 */
#define DRIVE_ERR_OVER_VOLTAGE_J0     70017
#define DRIVE_ERR_OVER_VOLTAGE_J1     70018
#define DRIVE_ERR_OVER_VOLTAGE_J2     70019
#define DRIVE_ERR_OVER_VOLTAGE_J3     70020
#define DRIVE_ERR_OVER_VOLTAGE_J4     70021
#define DRIVE_ERR_OVER_VOLTAGE_J5     70022


/**
 * @brief joint servo over temperature 关节伺服过温
 */
#define	DRIVE_ERR_DRIVER_OVERTEMP_J0 70031
#define	DRIVE_ERR_DRIVER_OVERTEMP_J1 70032
#define	DRIVE_ERR_DRIVER_OVERTEMP_J2 70033
#define	DRIVE_ERR_DRIVER_OVERTEMP_J3 70034
#define	DRIVE_ERR_DRIVER_OVERTEMP_J4 70035
#define	DRIVE_ERR_DRIVER_OVERTEMP_J5 70036


/**
 * @brief joint encoder error 关节编码器初始化失败
 */
#define DRIVE_ERR_ENC_AUTO_TURN_ERR_J0 70041
#define DRIVE_ERR_ENC_AUTO_TURN_ERR_J1 70042
#define DRIVE_ERR_ENC_AUTO_TURN_ERR_J2 70043
#define DRIVE_ERR_ENC_AUTO_TURN_ERR_J3 70044
#define DRIVE_ERR_ENC_AUTO_TURN_ERR_J4 70045
#define DRIVE_ERR_ENC_AUTO_TURN_ERR_J5 70046


/**
 * @brief joint failed to power on 关节上使能失败
 */
#define DRIVE_ERR_POWERON_FAILED_J0 70051
#define DRIVE_ERR_POWERON_FAILED_J1 70052
#define DRIVE_ERR_POWERON_FAILED_J2 70053
#define DRIVE_ERR_POWERON_FAILED_J3 70054
#define DRIVE_ERR_POWERON_FAILED_J4 70055
#define DRIVE_ERR_POWERON_FAILED_J5 70056



#define DRIVE_ERR_ECAT_MASTER       70100    //ECAT主站失败。
#define DRIVE_ERR_ECAT_DOMAIN       70101    //ECAT域失败。
#define DRIVE_ERR_ECAT_PDOS         70102    //ECAT PDO失败。
#define DRIVE_ERR_ECAT_RT_TASK      70103    //实时线程错误。


// ========================================================================================================================================================


// ==================================================================== 扩展IO模块错误代码 ===================================================================


#define	EXTIO_ERR_INIT_UNKNOWN 80001 //unknown error during initalization 初始化期间未知错误
#define	EXTIO_ERR_INIT_RTU_INCONSISTENT 80002 //rtu inconsistent during initialization 扩展IO初始化失败 MODBUS-RTU通讯参数不匹配
#define	EXTIO_ERR_INIT_TCP_FORMAT 80003 //tcp format error during initialization 扩展IO初始化失败 MODBUS-TCP通讯参数格式错误
#define	EXTIO_ERR_RUNTIME_RTU_OFFLINE 80004 //rtu node offline during runtime 扩展IO运行时失败 MODBUS-RTU节点断线
#define EXTIO_ERR_RUNTIME_TCP_OFFLINE 80005 //tcp node offline during runtime 扩展IO运行时失败 MODBUS-TCP节点断线


// ========================================================================================================================================================


// ==================================================================== 传感器模块错误代码 ===================================================================


#define	TORQSENSOR_ERR_FAILED_INIT_CONN 90001 //failed to initialize the connection with torque sensor 力矩传感器连接初始化失败
#define	TORQSENSOR_ERR_CANNOT_RECV_DATA 90002 //cannot receive data from torque sensor 接收力矩传感器数据失败
#define	TORQSENSOR_ERR_RECV_WRONG_DATA  90003 //wrong data is received from torque sensor 接收力矩传感器数据格式错误


// ========================================================================================================================================================

// ======================================================================== 其他错误 =======================================================================


#define UNKNOWN_ERROR  100000 //未知错误
#define ERROR_TEST_INT 100001 //测试错误

// ========================================================================================================================================================



}


#endif // ROBOTERRORCODE_H

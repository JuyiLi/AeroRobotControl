#ifndef ARCLOG_H
#define ARCLOG_H

/******************************************************************************
 * @file        arcLog.h
 * @brief       客户端日志指针。
 * @details     详细说明
 * @author      liurujia<1131970238@qq.com>
 * @date        2022-07-24
 * @version     v1.0
 * @copyright   Copyright By liu, All Rights Reserved.
 *
 ******************************************************************************
 *
 * <table>
 * @par 修改日志 Data: 2022-MM-dd Version: v1.0
 * @par 描述：
 * </table>
 *****************************************************************************/

#include "spdlog/spdlog.h"



/**
 * @brief log_ptr 客户端LOG的指针。
 */
extern std::shared_ptr<spdlog::logger> log_ptr;


/**
 * @brief setSpdLogger 配置客户端日志。Daily地址：~/ARCLOG/AeroRobot_Daily_Log.txt
 */
void setSpdLogger();


#endif // ARCLOG_H

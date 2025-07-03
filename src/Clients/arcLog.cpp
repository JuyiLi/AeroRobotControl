#include "arcLog.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"


/**
 * @brief log_ptr 客户端LOG的指针。
 */
std::shared_ptr<spdlog::logger> log_ptr;


/**
 * @brief setSpdLogger 配置客户端日志。Daily地址：~/ARCLOG/AeroRobot_Daily_Log.txt
 */
void setSpdLogger()
{
    /**
     * @brief fInit 初始化次数设置。
     */
    static bool fInit = false;
    if(fInit){
        return;
    }


    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);

    auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(std::string(getenv("HOME")) + "/ARCLOG/AeroRobot_Daily_Log.txt", 0, 0);
    daily_sink->set_level(spdlog::level::trace);

    spdlog::sinks_init_list sink_list = { daily_sink, console_sink };
    log_ptr = std::make_shared<spdlog::logger>("arcLog", sink_list.begin(), sink_list.end());

    log_ptr->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
    log_ptr->flush_on(spdlog::level::err);
    spdlog::register_logger(log_ptr);
    spdlog::flush_every(std::chrono::seconds(1));

    fInit = true;
}

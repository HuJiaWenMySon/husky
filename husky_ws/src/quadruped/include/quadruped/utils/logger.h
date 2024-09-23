#ifndef QUADRUPED_UTILS_LOGGER_H_
#define QUADRUPED_UTILS_LOGGER_H_

#include <cstdio>
#include <cstdarg>


namespace quadruped
{

/**
 * @brief 打印等级
 * 
 */
enum class LogLevel 
{
    DEBUG,
    INFO,
    WARN,
    ERROR
};


/**
 * @brief 日志类
 * 
 */
class Logger 
{
public:
    static inline void SetLevel(LogLevel level) {current_level_ = level;}
    static void Log(LogLevel level, const char* format, ...) ;

private:
    static LogLevel current_level_;
};



#define LOG_DEBUG(format, ...) Logger::Log(LogLevel::DEBUG, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...)  Logger::Log(LogLevel::INFO, format, ##__VA_ARGS__)
#define LOG_WARN(format, ...) Logger::Log(LogLevel::WARN, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) Logger::Log(LogLevel::ERROR, format, ##__VA_ARGS__)



}//namespace quadruped


#endif//QUADRUPED_UTILS_LOGGER_H_
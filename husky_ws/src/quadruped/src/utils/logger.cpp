#include "utils/logger.h"
#include <stdio.h>


namespace quadruped
{
    
LogLevel Logger::current_level_ = LogLevel::DEBUG;


/**
 * @brief 打印日志并换行
 * 
 * @param level 日志等级
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void Logger::Log(LogLevel level, const char* format, ...)
{
    if (level >= current_level_) 
    {
        switch (level) 
        {
        case LogLevel::DEBUG:
            printf("[ DEBUG] ");
            break;
        case LogLevel::INFO:
            printf("[ INFO] ");
            break;
        case LogLevel::WARN:
            printf("[ WARN] ");
            break;
        case LogLevel::ERROR:
            printf("[ ERROR] ");
            break;
        }

        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        printf("\n"); 
    }
}


}//namespace quadruped
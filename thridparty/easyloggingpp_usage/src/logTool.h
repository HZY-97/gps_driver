/**
 * @file logTool.h
 * @author huizeyu
 * @brief 
 * @version 0.1
 * @date 2023-07-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef LOG_TOOL_H
#define LOG_TOOL_H

#include <string>

#include "easylogging++.h"

extern std::string LOG_SAVE_PATH;

namespace Log
{

void rolloutHandler( const char *filename, std::size_t size );

bool InitLog( std::string configPath, std::string logSavePath, std::string logFileName );

class LogTool {
    /*func*/
public:
    LogTool() = delete;
    explicit LogTool( std::string loggerId );
    std::string m_loggerId;

protected:
private:
    /*data*/
public:
protected:
private:
};

} // namespace Log

#endif

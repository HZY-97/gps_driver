/**
 * @file logTool.cpp
 * @author huizeyu
 * @brief 
 * @version 0.1
 * @date 2023-07-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "logTool.h"

std::string LOG_SAVE_PATH;

namespace Log
{

void rolloutHandler( const char *filename, std::size_t size ) {
    // SHOULD NOT LOG ANYTHING HERE BECAUSE LOG FILE IS CLOSED!
    std::cout << "************** Rolling out [" << filename << "] because it reached [" << size << " bytes]"
              << std::endl;

    std::stringstream ss;

    std::string fileFullPath( filename );
    size_t      lastSlashPos = fileFullPath.find_last_of( '/' );
    if ( lastSlashPos != std::string::npos )
    {
        std::string fileNameStr = fileFullPath.substr( lastSlashPos + 1 );
        ss << "mv " << fileFullPath << " " + LOG_SAVE_PATH + "/Bak/Bak_" + fileNameStr;
        std::cout << ss.str() << std::endl;
        system( ss.str().c_str() );
    }
}

bool InitLog( std::string configPath, std::string logSavePath, std::string logFileName ) {
    LOG_SAVE_PATH   = logSavePath;
    std::string cmd = "mkdir -p " + logSavePath;
    system( cmd.c_str() );
    cmd = "mkdir -p " + logSavePath + "/Bak";
    system( cmd.c_str() );

    std::string logFileFullPath = logSavePath + "/" + logFileName;

    el::Configurations conf( configPath );
    conf.set( el::Level::Global, el::ConfigurationType::Filename, logFileFullPath.c_str() );
    el::Loggers::setDefaultConfigurations( conf );

    el::Loggers::addFlag( el::LoggingFlag::DisableApplicationAbortOnFatalLog );
    el::Loggers::addFlag( el::LoggingFlag::HierarchicalLogging );
    el::Loggers::addFlag( el::LoggingFlag::StrictLogFileSizeCheck );
    el::Loggers::addFlag( el::LoggingFlag::ColoredTerminalOutput );

    el::Helpers::installPreRollOutCallback( rolloutHandler );
}

LogTool::LogTool( std::string loggerId )
: m_loggerId( std::move( loggerId ) ) {
    el::Loggers::getLogger( m_loggerId );
}
} // namespace Log

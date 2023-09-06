#include "logTool.h"
#include <unistd.h>
INITIALIZE_EASYLOGGINGPP

int main( int argc, char **argv ) {
    std::string configPath =
        "/home/cat/code/RoboSLAM3/RoboSLAM3/src/RoboSLAM3/src/ThirdParty/easyloggingpp/config/Log.conf";
    std::string logSavePath = "/home/cat/code/RoboSLAM3/RoboSLAM3/src/RoboSLAM3/src/ThirdParty/easyloggingpp/build/Log";
    Log::InitLog( configPath, logSavePath, "MyTest1.log" );

    el::Loggers::getLogger( "testLogger" );
    while ( true )
    {
        CLOG( INFO, "testLogger" ) << "qweqwe";
        // usleep( 10 );
    }
    return 0;
}

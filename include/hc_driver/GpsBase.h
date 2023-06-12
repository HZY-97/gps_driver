#ifndef GPS_BASE_H
#define GPS_BASE_H

#include <memory>
#include <iostream>
#include <cstring>
#include <list>

#include "siasunLog.h"
#include "UdpSocketReceiver.h"

static const int BUFFER_SIZE = 4096;

class GpsBase {
public:
    explicit GpsBase(int port);
    virtual ~GpsBase();

    virtual bool InitUdp() = 0;

    virtual void DoReceive() = 0;

    char* GetBuffer(){return buffer;}

protected:
    std::shared_ptr <UdpSocketReceiver> m_udpSocketReceiver;
    char buffer[BUFFER_SIZE];
};

#endif  // GPS_BASE_H

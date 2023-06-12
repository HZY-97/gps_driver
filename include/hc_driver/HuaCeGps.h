#ifndef HUACE_GPS_H
#define HUACE_GPS_H

#include "GpsBase.h"

class HuaCeGps : public GpsBase
{
public:
    explicit HuaCeGps(int port);
    ~HuaCeGps();

    bool InitUdp() override;
    void DoReceive() override;

    void PrintData();

    int GetSysState();

    int GetSatelliteState();

private:
    void Decode();

    int FindSyncHead();

    bool charsIsNMEA(std::string s);

    int checksum(std::string s);

    void Decompose(std::string &Message);

    void Analysis();

public:
    // DATA
    int m_iGPSWeek;
    double m_dGPSTime;
    double m_dHeading;
    double m_dPitch;
    double m_dRoll;
    double m_dgyrox;
    double m_dgyroy;
    double m_dgyroz;
    double m_daccx;
    double m_daccy;
    double m_daccz;
    double m_dLattitude;
    double m_dLongitude;
    double m_dAltitude;
    double m_dVe;
    double m_dVn;
    double m_dVu;
    double m_dV;
    int m_iNSV1;
    int m_iNSV2;
    int m_iStatus;
    int m_iAge;
    int m_iWarmingCs;

private:
    std::string MessageID;
    std::string Message;
    std::list<std::string> component;
};

#endif // HUACE_GPS_H

#pragma once 

#include "Comnav_Message.hpp"
#include <string>
#include <list>
#include "crc.h"
using namespace std;
class GNSS_NMEAMessage : public GNSS_Message2
{public:
    string MessageID;
    GNSS_NMEAMessage(string Message) : GNSS_Message2(Message)
    {
        MaxLength = 1 * 1024;
        Decode();
    };
    virtual ~GNSS_NMEAMessage() {};
    list<string> component;
    int FindSyncHead() override
    {
        return Message.find("$",0);
    }

    bool charsIsNMEA(string s)
    {
        for(int i=0;i<(int)s.length();i++)
        {
            if ( (s[i]>=32 && s[i]<=127) || s[i]==0x0d || s[i]==0x0a )
            {}
            else
                return false;
        }
        return true;
    }

    int checksum(string s)
    {
        unsigned char chksum = 0;
        for (int i=1; i<s.length(); i++)
        {
            chksum ^= s[i];
        }
        return chksum;
    }

    Comnav_Decode_Error _Decode(string& Message) override
    {
        if(Message.length() < 2)
            return Comnav_Decode_MessageShort;

        int starpos = Message.find("*", 0);
        if( (starpos==-1) && (!charsIsNMEA(Message)) )
            return Comnav_Decode_DecodeError;
        if( (starpos==-1) && (charsIsNMEA(Message)) )
            return Comnav_Decode_MessageShort;
        if( (starpos!=-1) && (!charsIsNMEA(Message.substr(0,starpos))) )
            return Comnav_Decode_DecodeError;

        char endlstr[1] = {0x0A};
        int endlpos = Message.find( string().append( endlstr, 1), starpos);
        if((Message.length() - starpos > 5) && (endlpos == -1))
            return Comnav_Decode_DecodeError;
        if(endlpos == -1)
            return Comnav_Decode_MessageShort;


        string Body = Message.substr(1,starpos-1);
        if( Body.find("$",0) != Body.npos ) return Comnav_Decode_DecodeError;

        int pos1 = 0, pos2;
        while( pos1 <= (int)Body.length() )
        {
            pos2 = Body.find(",", pos1);

            if(pos2 == (int)Body.npos)
                pos2 = Body.length();

            component.push_back(
                string(Body.substr(pos1, pos2-pos1))
            );
            //DEBUG_INFO("%s",string(Body.substr(pos1, pos2-pos1)).c_str());
            pos1 = pos2 + 1;
        };
        MessageID = component.front();
        Message = Message.substr(0, endlpos + 1);

        /*printf("Message: %s\r\n", Message.c_str());
        printf("Message: %s\r\n", Message.substr(0, starpos).c_str());
        printf("ChecksumCalc: %d\r\n", checksum(Message.substr(0, starpos)));
        printf("checksumGet: %s\r\n", Message.substr(starpos+1, 2).c_str());
        printf("ChecksumGet: %d\r\n", stoi(Message.substr(starpos+1, 2),0,16));*/

        if(checksum(Message.substr(0, starpos)) != stoi(Message.substr(starpos+1, 2),0,16))
        {
            return Comnav_Decode_DecodeError;
        }

        return Comnav_Decode_Success;
    }
};


class GNSS_GPGGAMessage : public GNSS_NMEAMessage
{public:
    GNSS_GPGGAMessage():GNSS_NMEAMessage(""){}
    GNSS_GPGGAMessage(const GNSS_NMEAMessage& p) : GNSS_NMEAMessage(p.Message) { Decode();  }

    float  utc;
    double latitude_Dm;
    char   latdir;
    double longitude_Dm;
    char   londir;
    int    fix_status;
    int    fix_satellite;
    float  hdop;
    double altitude;
    char   alt_unit;
    float  undulation;
    char   und_unit;
    int    age;
    string station_id;

    double latitude_D;
    double longitude_D;

    double GGA_degree(double x, char d)//3130.5 -> 31.500139
    {
        int degree = (int)x / 100;
        double minute = x - degree*100;
        double ret = degree + minute /60;
        if(d == 'W' || d == 'S') ret=-ret;
        return ret;
    }

    Comnav_Decode_Error _Decode(string& Message) override
    {
        list<string>::iterator iter1 = component.begin();
        iter1++;
        utc           = atof((*iter1++).c_str());
        latitude_Dm   = atof((*iter1++).c_str());
        latdir        = *(*iter1++).c_str();
        longitude_Dm  = atof((*iter1++).c_str());
        londir        = *(*iter1++).c_str();
        fix_status    = atoi((*iter1++).c_str());
        fix_satellite = atof((*iter1++).c_str());
        hdop          = atof((*iter1++).c_str());
        altitude      = atof((*iter1++).c_str());
        alt_unit      = *(*iter1++).c_str();
        undulation    = atof((*iter1++).c_str());
        und_unit      = *(*iter1++).c_str();
        age           = atof((*iter1++).c_str());
        station_id    = *iter1++;

        latitude_D  = GGA_degree(latitude_Dm, latdir);
        longitude_D = GGA_degree(longitude_Dm, londir);
        return Comnav_Decode_Success;
    }
};

class GNSS_GPYBMMessage : public GNSS_NMEAMessage
{public:
    GNSS_GPYBMMessage():GNSS_NMEAMessage(""){}
    GNSS_GPYBMMessage(const GNSS_NMEAMessage& p) : GNSS_NMEAMessage(p.Message) { Decode();  }

    string SN;
    float  utc;
    double latitude_D;
    double longitude_D;
    double altitude;
    double pow;
    double pitch;
    double speedX;
    double speedY;
    double speedZ;
    double groundspeed;
    double GaussX;
    double GaussY;
    double baseX;
    double baseY;
    int    fix_status;
    int    fix_status2;
    int    fix_satellite;
    int delay;
    string StationID;
    int    BaseDist;
    int    Slave_fix_status;
    double roll;

    Comnav_Decode_Error _Decode(string& Message) override
    {
        list<string>::iterator iter1 = component.begin();
        iter1++;
        SN               = *iter1++;
        utc              = atof((*iter1++).c_str());
        latitude_D       = atof((*iter1++).c_str());
        longitude_D      = atof((*iter1++).c_str());
        altitude         = atof((*iter1++).c_str());
        pow              = atof((*iter1++).c_str());
        pitch            = atof((*iter1++).c_str());
        speedX           = atof((*iter1++).c_str());
        speedY           = atof((*iter1++).c_str());
        speedZ           = atof((*iter1++).c_str());
        groundspeed      = atof((*iter1++).c_str());
        GaussX           = atof((*iter1++).c_str());
        GaussY           = atof((*iter1++).c_str());
        baseX            = atof((*iter1++).c_str());
        baseY            = atof((*iter1++).c_str());
        fix_status       = atof((*iter1++).c_str());
        fix_status2      = atof((*iter1++).c_str());
        fix_satellite    = atof((*iter1++).c_str());
        delay            = atof((*iter1++).c_str());
        StationID        = *iter1++;
        BaseDist         = atof((*iter1++).c_str());
        Slave_fix_status = atof((*iter1++).c_str());
        roll             = atof((*iter1++).c_str());
        return Comnav_Decode_Success;
    }
};

class GNSS_GPCHCMessage : public GNSS_NMEAMessage
{
    public:
    GNSS_GPCHCMessage():GNSS_NMEAMessage(""){}
    GNSS_GPCHCMessage(const GNSS_NMEAMessage& p) : GNSS_NMEAMessage(p.Message) { Decode();  }

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

    Comnav_Decode_Error _Decode(string& Message) override
    {
        list<string>::iterator iter1 = component.begin();
        iter1++;

        m_iGPSWeek = atoi((*iter1++).c_str());
        m_dGPSTime = atof((*iter1++).c_str());
        m_dHeading = atof((*iter1++).c_str());
        m_dPitch = atof((*iter1++).c_str());
        m_dRoll = atof((*iter1++).c_str());
        m_dgyrox = atof((*iter1++).c_str());
        m_dgyroy = atof((*iter1++).c_str());
        m_dgyroz = atof((*iter1++).c_str());
        m_daccx = atof((*iter1++).c_str());
        m_daccy = atof((*iter1++).c_str());
        m_daccz = atof((*iter1++).c_str());
        m_dLattitude = atof((*iter1++).c_str());
        m_dLongitude = atof((*iter1++).c_str());
        m_dAltitude = atof((*iter1++).c_str());
        m_dVe = atof((*iter1++).c_str());
        m_dVn = atof((*iter1++).c_str());
        m_dVu = atof((*iter1++).c_str());
        m_dV = atof((*iter1++).c_str());
        m_iNSV1 = atoi((*iter1++).c_str());
        m_iNSV2 = atoi((*iter1++).c_str());
        m_iStatus = atoi((*iter1++).c_str());
        m_iAge = atoi((*iter1++).c_str());

        return Comnav_Decode_Success;
    }
};
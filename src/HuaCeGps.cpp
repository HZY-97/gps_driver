#include "HuaCeGps.h"

HuaCeGps::HuaCeGps(int port) : GpsBase(port)
{
    // Initialize HuaCeGps specific members
    m_iGPSWeek = 0;
    m_dGPSTime = 0.0;
    m_dHeading = 0.0;
    m_dPitch = 0.0;
    m_dRoll = 0.0;
    m_dgyrox = 0.0;
    m_dgyroy = 0.0;
    m_dgyroz = 0.0;
    m_daccx = 0.0;
    m_daccy = 0.0;
    m_daccz = 0.0;
    m_dLattitude = 0.0;
    m_dLongitude = 0.0;
    m_dAltitude = 0.0;
    m_dVe = 0.0;
    m_dVn = 0.0;
    m_dVu = 0.0;
    m_dV = 0.0;
    m_iNSV1 = 0;
    m_iNSV2 = 0;
    m_iStatus = 0;
    m_iAge = 0;
    m_iWarmingCs = 0;

    // Initialize UDP socket receiver
    m_udpSocketReceiver = std::make_shared<UdpSocketReceiver>(port);
}

HuaCeGps::~HuaCeGps() {}

bool HuaCeGps::InitUdp()
{
    // Initialize UDP socket and other necessary configurations
    if (!m_udpSocketReceiver->Initialize())
    {
        std::cerr << "Failed to initialize UDP socket" << std::endl;
        return false;
    }

    return true;
}

void HuaCeGps::DoReceive()
{
    if (m_udpSocketReceiver->ReceiveData(buffer, BUFFER_SIZE))
    {
        Message = buffer;
        std::memset(buffer, 0, BUFFER_SIZE);
        // std::cout << "Received data: " << buffer << std::endl;
        Decode();
    }
};

void HuaCeGps::PrintData()
{
    std::cout << "******************************" << std::endl;
    std::cout << "m_iGPSWeek = " << m_iGPSWeek << std::endl;
    std::cout << "m_dGPSTime = " << m_dGPSTime << std::endl;
    std::cout << "m_dHeading = " << m_dHeading << std::endl;
    std::cout << "m_dPitch = " << m_dPitch << std::endl;
    std::cout << "m_dRoll = " << m_dRoll << std::endl;
    std::cout << "m_dgyrox = " << m_dgyrox << std::endl;
    std::cout << "m_dgyroy = " << m_dgyroy << std::endl;
    std::cout << "m_dgyroz = " << m_dgyroz << std::endl;
    std::cout << "m_daccx = " << m_daccx << std::endl;
    std::cout << "m_daccy = " << m_daccy << std::endl;
    std::cout << "m_daccz = " << m_daccz << std::endl;
    std::cout << "m_dLattitude = " << m_dLattitude << std::endl;
    std::cout << "m_dLongitude = " << m_dLongitude << std::endl;
    std::cout << "m_dAltitude = " << m_dAltitude << std::endl;
    std::cout << "m_dVe = " << m_dVe << std::endl;
    std::cout << "m_dVn = " << m_dVn << std::endl;
    std::cout << "m_dVu = " << m_dVu << std::endl;
    std::cout << "m_dV = " << m_dV << std::endl;
    std::cout << "m_iNSV1 = " << m_iNSV1 << std::endl;
    std::cout << "m_iNSV2 = " << m_iNSV2 << std::endl;
    std::cout << "m_iStatus = " << m_iStatus << std::endl;
    std::cout << "m_iAge = " << m_iAge << std::endl;
    std::cout << "m_iWarmingCs = " << m_iWarmingCs << std::endl;
}

int HuaCeGps::GetSysState()
{
    if (m_iStatus < 10)
    {
        return 0;
    }
    else
    {
        return m_iStatus % 10;
    }
}

int HuaCeGps::GetSatelliteState()
{
    if (m_iStatus < 10)
    {
        return m_iStatus;
    }
    else
    {
        return (m_iStatus / 10) % 10;
    }
}

/**********************************PRIVATE***********************************/

void HuaCeGps::Decode()
{
    Decompose(Message);
    Analysis();
}

int HuaCeGps::FindSyncHead()
{
    return Message.find("$", 0);
}

bool HuaCeGps::charsIsNMEA(std::string s)
{
    for (int i = 0; i < (int)s.length(); i++)
    {
        if ((s[i] >= 32 && s[i] <= 127) || s[i] == 0x0d || s[i] == 0x0a)
        {
        }
        else
            return false;
    }
    return true;
}

int HuaCeGps::checksum(std::string s)
{
    unsigned char chksum = 0;
    for (int i = 1; i < s.length(); i++)
    {
        chksum ^= s[i];
    }
    return chksum;
}

void HuaCeGps::Decompose(std::string &Message)
{
    component.clear();
    if (Message.length() < 2)
        return;

    int starpos = Message.find("*", 0);
    if ((starpos == -1) && (!charsIsNMEA(Message)))
        return;
    if ((starpos == -1) && (charsIsNMEA(Message)))
        return;
    if ((starpos != -1) && (!charsIsNMEA(Message.substr(0, starpos))))
        return;

    char endlstr[1] = {0x0A};
    int endlpos = Message.find(std::string().append(endlstr, 1), starpos);
    if ((Message.length() - starpos > 5) && (endlpos == -1))
        return;
    if (endlpos == -1)
        return;

    std::string Body = Message.substr(1, starpos - 1);
    if (Body.find("$", 0) != Body.npos)
        return;

    int pos1 = 0, pos2;
    while (pos1 <= (int)Body.length())
    {
        pos2 = Body.find(",", pos1);

        if (pos2 == (int)Body.npos)
            pos2 = Body.length();

        component.push_back(
            std::string(Body.substr(pos1, pos2 - pos1)));
        pos1 = pos2 + 1;
    };
    MessageID = component.front();
    Message = Message.substr(0, endlpos + 1);

    if (checksum(Message.substr(0, starpos)) != stoi(Message.substr(starpos + 1, 2), 0, 16))
    {
        return;
    }

    return;
}

void HuaCeGps::Analysis()
{
    // std::cout<<"MessageID: "<< MessageID << std::endl;
    if("GPCHC" != MessageID)
    {
        return;
    }
    if (component.empty())
    {
        return;
    }
    std::list<std::string>::iterator iter1 = component.begin();
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
}
#pragma once
#include <string>

#include "Comnav_BinaryCoder.hpp"
#include "Comnav_Message.hpp"
// #include "crc.h"

using namespace std;
class GNSS_NovatelMessage : public GNSS_Message2 {
 public:
  int MessageID;
  GNSS_NovatelMessage(string Message) : GNSS_Message2(Message) {
    SyncHeadLength = 3;
    MaxLength = 8 * 1024;
    Decode();
  };
  virtual ~GNSS_NovatelMessage(){};

  int GPSWeek;
  int GPSms;

  int FindSyncHead() override {
    unsigned char synchead[3] = {0xaa, 0x44, 0x12};
    // DEBUG_INFO("GNSS_NovatelMessage::FindSyncHead() == %d",
    // Message.find(string().append((char*)synchead,3),0));
    return Message.find(string().append((char*)synchead, 3), 0);
  }

  Comnav_Decode_Error _Decode(string& Message) override {
    if (Message.length() < 28)
      return Comnav_Decode_MessageShort;
    Comnav_BinaryDecoder decoder(Message);
    MessageID = decoder.Decode_Word(4);
    if ((int)MessageID > 9999)
      return Comnav_Decode_DecodeError;
    int Length = decoder.Decode_Word(8);
    if (Length > 19999)
      return Comnav_Decode_DecodeError;
    if ((int)Message.length() < Length + 32)
      return Comnav_Decode_MessageShort;
    GPSWeek = decoder.Decode_Word(14);
    GPSms = decoder.Decode_Int(16);
    Message = Message.substr(0, Length + 32);

    // int crc = CalcMsgCRC((char*)Message.c_str(), Message.length());
    // DEBUG_INFO("crc = %d", crc);
    // if (crc != 0) {
    //   return Comnav_Decode_DecodeError;
    // }
    return Comnav_Decode_Success;
  }
};

class GNSS_BESTPOSB : public GNSS_NovatelMessage {
 public:
  GNSS_BESTPOSB() : GNSS_NovatelMessage("") {
  }
  GNSS_BESTPOSB(const GNSS_NovatelMessage& p) : GNSS_NovatelMessage(p.Message) {
    Decode();
  }

  unsigned short uGpsWeek;
  double fGpsWeekSecond;
  unsigned int sol_stat;
  unsigned int pos_type;
  double lat;
  double lon;
  double hgt;
  unsigned char SVs;
  unsigned char sonlnSVs;

  Comnav_Decode_Error _Decode(string& Message) override {
    Comnav_BinaryDecoder decoder(Message);
    GPSms = decoder.Decode_Word(14);
    fGpsWeekSecond = decoder.Decode_Int(16) / 1000.0;
    sol_stat = decoder.Decode_Int(28);
    pos_type = decoder.Decode_Int(32);
    lat = decoder.Decode_Double(36);
    lon = decoder.Decode_Double(44);
    hgt = decoder.Decode_Double(52);
    SVs = decoder.Decode_Char(32);
    sonlnSVs = decoder.Decode_Char(33);
    return Comnav_Decode_Success;
  }
};

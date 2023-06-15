#pragma once
#ifdef __cplusplus
#include <iostream>
using namespace std;

/** Comnav_Message
 * @brief 报文/消息类
 * @details 派生出发送消息和接收消息
 *
 * @todo
 * @author zyf
 * @date 2022.08.10
 */
class Comnav_Message
{
    protected: Comnav_Message() {};
    private:   Comnav_Message(string Message): Message(Message) {};
    public:    string Message;
};

/*发送报文*/
class Comnav_Send_Message : virtual public Comnav_Message
{
    public:  Comnav_Send_Message() {};

    public: void Encode()
    {
        Message = _Encode();
    }

    public: virtual string _Encode()=0;
};

class Comnav_Send_Protocol_Message : virtual public Comnav_Send_Message
{public:
    Comnav_Send_Protocol_Message() {};

    string Body;
    string Head;
    string Tail;

    public: void Encode()
    {
        Body = _Encode();
        Head = EncodeHead();
        Tail = EncodeTail();
        Message = Head + Body + Tail;
    }

    virtual string EncodeHead() {return "";};
    virtual string EncodeTail() {return "";};
};

/*接收报文解析错误码*/
enum Comnav_Decode_Error{
    Comnav_Decode_Success            = 1,
    Comnav_Decode_UnknownError       = -100,
    Comnav_Decode_NoMessage          = -101,
    Comnav_Decode_NotStart           = -102,
    Comnav_Decode_Exception          = -103,
    Comnav_Decode_SyncHeadShort      = -200,
    Comnav_Decode_SyncHeadCannotFind = -201,
    Comnav_Decode_MessageShort       = -300,
    Comnav_Decode_DecodeError        = -301,
};

/*接收报文*/
class Comnav_Recv_Message : virtual public Comnav_Message
{
    public: Comnav_Recv_Message(string Message) {this->Message = Message;};

    public:Comnav_Decode_Error DecodeResult = Comnav_Decode_NotStart;
    public: bool isSuccess(){return (DecodeResult==Comnav_Decode_Success);};

    public: string Message_Remained;

    public: int SyncHeadLength = 1;
    private:virtual int FindSyncHead() { return 0; };
    public: int MaxLength = 32*1024;
    private:virtual Comnav_Decode_Error _Decode(string& Message)=0;

    public: void Decode()
    {
        DecodeResult = Comnav_Decode_NotStart;
        DecodeResult = DecodeGetResult();
    }

    private:Comnav_Decode_Error DecodeGetResult()
    {
        try
        {
            start:
            if(Message.length() == 0)
            {
                return Comnav_Decode_NoMessage;
            }
            if((int)Message.length() > MaxLength + 16 * 1024)
            {
                Message = Message.substr(MaxLength + 16 * 1024, Message.length() - (MaxLength + 16 * 1024) );
            }
            //DEBUG_INFO("Message.length() == %d",Message.length());
            if((int)Message.length() < SyncHeadLength)
            {
                Message_Remained = Message;
                return Comnav_Decode_SyncHeadShort;
            }
            //DEBUG_INFO("FindSyncHead() == %d",FindSyncHead());
            if(FindSyncHead() == -1)
            {
                Message_Remained = Message.substr(Message.length()-SyncHeadLength, SyncHeadLength);
                return Comnav_Decode_SyncHeadCannotFind;
            }
            else
            {
                //DEBUG_INFO("Message.length() == %d",Message.length());
                //DEBUG_INFO("FindSyncHead() == %d",FindSyncHead());
                Message.erase(0, FindSyncHead());
            }
            //DEBUG_INFO("Message.length() == %d",Message.length());
            //DEBUG_INFO("Message[0] == %x", Message[0]);
            //DEBUG_INFO("Message[1] == %x", Message[1]);
            //Message_Remained=Message;
            string Message2 = Message;
            Comnav_Decode_Error ret = _Decode(Message2);
            //DEBUG_PRINT_STRING(Message_Remained);
            //DEBUG_INFO("ret == %d", ret);
            //DEBUG_INFO("111 Message == %d", Message.length());


            if(ret == Comnav_Decode_Success)
            {
                //Total_Length = Message.length();
                //Message_Remained = Message.substr(Total_Length,Message.length() - Total_Length);
                Message_Remained = Message.erase(0, Message2.length());
                Message = Message2;
                //Message = Message.substr(0,Total_Length);
            }
            else if (ret == Comnav_Decode_DecodeError)
            {
                //Message = Message_Remained;
                //DEBUG_INFO("222 Message == %d", Message.length());
                Message.erase(0,1);
                //DEBUG_INFO("333 Message == %d", Message.length());
                goto start;
            }
            else if((int)Message.length() > MaxLength)
            {
                //Message = Message_Remained;
                //DEBUG_ERR("Message.length() %d > %d" ,(int)Message.length(),  MaxLength);
                Message.erase(0,1);
                goto start;
            }
            else if(ret == Comnav_Decode_MessageShort)
            {
                Message_Remained = Message;
                //DEBUG_INFO("Message_Remained.length() == %d",Message_Remained.length());
                return Comnav_Decode_MessageShort;
            }
            else
            {
                Message_Remained = "";
                return ret;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return Comnav_Decode_Exception;
        }
        catch(...)
        {
            return Comnav_Decode_UnknownError;
        }

        return Comnav_Decode_Success;
    };
};
class GNSS_Message2 : public Comnav_Recv_Message
{public:
    GNSS_Message2(string Message)
        : Comnav_Recv_Message(Message) {};
    virtual ~GNSS_Message2() {};
};
#endif

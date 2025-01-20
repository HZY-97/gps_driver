#pragma once
#ifdef __cplusplus
#include <string.h>

#include <string>

using namespace std;

/** Comnav_BinaryDecoder
 * @brief 二进制消息解码工具
 *        使用 构造函数(string) 将需要解码的消息传入
 *        使用 Decode_xxx(value)
 * 将字节按照指定类型取出，xxx代表数据类型，默认小端，大端使用带Encode_xxx_Reverse(value)的方法
 * @details 支持解码int char string word long float
 * double等，或使用Decode()打包任意类型数据
 *          消息中的预留字节可以使用Encode_Empty(length)跳过
 *
 * @todo Message[65536]定义过大，占用空间多，可以改为动态分配
 * @author zyf
 * @date 2022.08.10
 */
class Comnav_BinaryDecoder {
 public:
  /**
   * @fn void Comnav_BinaryDecoder(string Message)
   * @brief 打包数据
   * @details 类的构造函数
   * @param Message 需要解析的数据
   */
  Comnav_BinaryDecoder(string Message) {
    memcpy(this->Message, Message.c_str(), Message.length());
  };

  int index = 0;
  char Message[65536];

  /**
   * @fn void Decode(int Offset, int length, char* buff)
   * @brief 打包数据
   * @details 所有Decode_xxx_xxx()函数最终调用该函数
   * @param Offset 当前解析到的到的位置
   * @param length 数据长度
   * @param buff 数据头指针
   */
  void Decode(int Offset, int length, char* buff) {
    memcpy(buff, &Message[Offset], length);
    index = Offset + length;
  }

  /**
   * @fn void Decode_xxx_xxx(value)
   * @brief 解析数据
   * @details 函数名中xxx代表需要打包的数据类型，默认小端，带Reverse表示大端
   * @param value 解析后的值
   */
  void reverse_byte(char* c, char num) {
    int i;
    for (i = 0; i < num / 2; i++) {
      char t = c[i];
      c[i] = c[num - 1 - i];
      c[num - 1 - i] = t;
    }
  }
  int Decode_Int() {
    int value = 0;
    Decode(index, 4, (char*)&value);
    return value;
  }
  int Decode_Int_Reverse() {
    int value = 0;
    reverse_byte(&Message[index], 4);
    Decode(index, 4, (char*)&value);
    return value;
  }
  int Decode_Int(int index) {
    this->index = index;
    return Decode_Int();
  }
  int Decode_Int_Reverse(int index) {
    this->index = index;
    return Decode_Int_Reverse();
  }
  string Decode_String(int length) {
    char value[512] = {0};
    Decode(index, length, (char*)value);
    return string().append(value, length);
  }
  string Decode_String(int index, int length) {
    this->index = index;
    return Decode_String(length);
  }
  char Decode_Char() {
    char value = 0;
    Decode(index, 1, &value);
    return value;
  }
  char Decode_Char(int index) {
    this->index = index;
    return Decode_Char();
  }
  unsigned char Decode_UnsignedChar() {
    unsigned char value = 0;
    Decode(index, 1, (char*)&value);
    return value;
  }
  unsigned char Decode_UnsignedChar(int index) {
    this->index = index;
    return Decode_UnsignedChar();
  }
  int Decode_Word() {
    int value = 0;
    Decode(index, 2, (char*)&value);
    return value;
  }
  int Decode_Word(int index) {
    this->index = index;
    return Decode_Word();
  }
  int Decode_Word_Reverse() {
    int value = 0;
    reverse_byte(&Message[index], 2);
    Decode(index, 2, (char*)&value);
    return value;
  }
  int Decode_Word_Reverse(int index) {
    this->index = index;
    return Decode_Word_Reverse();
  }
  long long int Decode_Long() {
    long long int value = 0;
    Decode(index, 8, (char*)&value);
    return value;
  }
  long long int Decode_Long(int index) {
    this->index = index;
    return Decode_Long();
  }
  long long int Decode_Long_Reverse() {
    long long int value = 0;
    reverse_byte(&Message[index], 8);
    Decode(index, 8, (char*)&value);
    return value;
  }
  long long int Decode_Long_Reverse(int index) {
    this->index = index;
    return Decode_Long_Reverse();
  }
  float Decode_Float() {
    float value = 0.0;
    Decode(index, 4, (char*)&value);
    return value;
  }
  float Decode_Float(int index) {
    this->index = index;
    return Decode_Float();
  }
  float Decode_Float_Reverse() {
    float value = 0.0;
    reverse_byte(&Message[index], 4);
    Decode(index, 4, (char*)&value);
    return value;
  }
  float Decode_Float_Reverse(int index) {
    this->index = index;
    return Decode_Float_Reverse();
  }
  double Decode_Double() {
    double value = 0.0;
    Decode(index, 8, (char*)&value);
    return value;
  }
  double Decode_Double(int index) {
    this->index = index;
    return Decode_Double();
  }
  double Decode_Double_Reverse() {
    double value = 0.0;
    reverse_byte(&Message[index], 8);
    Decode(index, 8, (char*)&value);
    return value;
  }
  double Decode_Double_Reverse(int index) {
    this->index = index;
    return Decode_Double_Reverse();
  }
  void Decode_Empty(int length) {
    index += length;
  }
};

#endif

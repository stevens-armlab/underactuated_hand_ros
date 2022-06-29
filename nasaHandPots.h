#ifndef _ROS_nasa_hand_msgs_nasaHandPots_h
#define _ROS_nasa_hand_msgs_nasaHandPots_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nasa_hand_msgs
{

  class nasaHandPots : public ros::Msg
  {
    public:
      typedef float _pot1_type;
      _pot1_type pot1;
      typedef float _pot2_type;
      _pot2_type pot2;
      typedef float _pot3_type;
      _pot3_type pot3;
      typedef float _pot4_type;
      _pot4_type pot4;
      typedef float _pot5_type;
      _pot5_type pot5;
      typedef float _pot6_type;
      _pot6_type pot6;
      typedef float _pot7_type;
      _pot7_type pot7;
      typedef float _pot8_type;
      _pot8_type pot8;

    nasaHandPots():
      pot1(0),
      pot2(0),
      pot3(0),
      pot4(0),
      pot5(0),
      pot6(0),
      pot7(0),
      pot8(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pot1;
      u_pot1.real = this->pot1;
      *(outbuffer + offset + 0) = (u_pot1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pot1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pot1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pot1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pot1);
      union {
        float real;
        uint32_t base;
      } u_pot2;
      u_pot2.real = this->pot2;
      *(outbuffer + offset + 0) = (u_pot2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pot2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pot2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pot2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pot2);
      union {
        float real;
        uint32_t base;
      } u_pot3;
      u_pot3.real = this->pot3;
      *(outbuffer + offset + 0) = (u_pot3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pot3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pot3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pot3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pot3);
      union {
        float real;
        uint32_t base;
      } u_pot4;
      u_pot4.real = this->pot4;
      *(outbuffer + offset + 0) = (u_pot4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pot4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pot4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pot4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pot4);
      union {
        float real;
        uint32_t base;
      } u_pot5;
      u_pot5.real = this->pot5;
      *(outbuffer + offset + 0) = (u_pot5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pot5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pot5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pot5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pot5);
      union {
        float real;
        uint32_t base;
      } u_pot6;
      u_pot6.real = this->pot6;
      *(outbuffer + offset + 0) = (u_pot6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pot6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pot6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pot6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pot6);
      union {
        float real;
        uint32_t base;
      } u_pot7;
      u_pot7.real = this->pot7;
      *(outbuffer + offset + 0) = (u_pot7.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pot7.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pot7.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pot7.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pot7);
      union {
        float real;
        uint32_t base;
      } u_pot8;
      u_pot8.real = this->pot8;
      *(outbuffer + offset + 0) = (u_pot8.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pot8.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pot8.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pot8.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pot8);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pot1;
      u_pot1.base = 0;
      u_pot1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pot1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pot1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pot1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pot1 = u_pot1.real;
      offset += sizeof(this->pot1);
      union {
        float real;
        uint32_t base;
      } u_pot2;
      u_pot2.base = 0;
      u_pot2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pot2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pot2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pot2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pot2 = u_pot2.real;
      offset += sizeof(this->pot2);
      union {
        float real;
        uint32_t base;
      } u_pot3;
      u_pot3.base = 0;
      u_pot3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pot3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pot3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pot3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pot3 = u_pot3.real;
      offset += sizeof(this->pot3);
      union {
        float real;
        uint32_t base;
      } u_pot4;
      u_pot4.base = 0;
      u_pot4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pot4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pot4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pot4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pot4 = u_pot4.real;
      offset += sizeof(this->pot4);
      union {
        float real;
        uint32_t base;
      } u_pot5;
      u_pot5.base = 0;
      u_pot5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pot5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pot5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pot5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pot5 = u_pot5.real;
      offset += sizeof(this->pot5);
      union {
        float real;
        uint32_t base;
      } u_pot6;
      u_pot6.base = 0;
      u_pot6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pot6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pot6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pot6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pot6 = u_pot6.real;
      offset += sizeof(this->pot6);
      union {
        float real;
        uint32_t base;
      } u_pot7;
      u_pot7.base = 0;
      u_pot7.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pot7.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pot7.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pot7.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pot7 = u_pot7.real;
      offset += sizeof(this->pot7);
      union {
        float real;
        uint32_t base;
      } u_pot8;
      u_pot8.base = 0;
      u_pot8.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pot8.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pot8.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pot8.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pot8 = u_pot8.real;
      offset += sizeof(this->pot8);
     return offset;
    }

    const char * getType(){ return "nasa_hand_msgs/nasaHandPots"; };
    const char * getMD5(){ return "b781ea501a6858afdb002ed8892ad06e"; };

  };

}
#endif

#ifndef R2FGRIPPER_H
#define R2FGRIPPER_H

#include <ros/ros.h>
#include <string>

#include <irl_robots/r2fgControl.h>
#include <irl_robots/gSimpleControl.h>

#define SLAVE_ID 0x09
#define PRESET_MULTIPLE_REGISTERS 0x10
#define READ_HOLDING_REGISTERS 0x03
#define READ_AND_WRITE_MULTIPLE_REGISTERS 0x17

typedef struct __attribute__ ((aligned(8)))
{
    unsigned char gOBJ;
    unsigned char gSTA;
    unsigned char gGTO;
    unsigned char gACT;
    unsigned char kFLT;
    unsigned char gFLT;
    unsigned char gPR;
    unsigned char gPO;
    unsigned char gCU;
} GInput;

typedef struct __attribute__ ((aligned(8)))
{
    unsigned char rARD;
    unsigned char rATR;
    unsigned char rGTO;
    unsigned char rACT;
    unsigned char rPR;
    unsigned char rSP;
    unsigned char rFR;
} GOutput;

class R2FGripper
{
  public:
    R2FGripper();
    ~R2FGripper();
    void connectGripper();

  private:
    unsigned short computeCRC(unsigned char* adr, unsigned short length);
    void createCommand(unsigned char* data);
    void parseState(unsigned char* data);
    void getValues();
    void serialWrite(unsigned char* data, size_t size);
    void serialRead(unsigned char** buffer, unsigned short size);
    void setValues();
    void printData(unsigned char* data, size_t size);
    void setSerialAttributes();

    void callbackControl(irl_robots::r2fgControl c);
    void callbackSimpleControl(irl_robots::gSimpleControl c);

    int m_serial_handle;
    std::string m_serial_port;
    ros::NodeHandle m_ros_node;
    ros::Subscriber m_sub_control;
    ros::Subscriber m_sub_control2;
    ros::Publisher m_pub_status;
    GInput m_gripper_input;
    GOutput m_gripper_output;
};

#endif // R2FGRIPPER_H

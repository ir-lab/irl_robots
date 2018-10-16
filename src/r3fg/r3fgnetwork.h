#ifndef R3FGNETWORK_H
#define R3FGNETWORK_H

#include <ros/ros.h>
#include <thread>
#include <arpa/inet.h>
#include <mutex>
#include <queue>

#include <irl_robots/r3fgControl.h>
#include <irl_robots/gSimpleControl.h>

typedef struct __attribute__ ((aligned (8))) _rgo
{
  unsigned char rACT;
  unsigned char rMOD;
  unsigned char rGTO;
  unsigned char rATR;
  unsigned char rGLV;
  unsigned char rICF;
  unsigned char rICS;
  unsigned char rPRA;
  unsigned char rSPA;
  unsigned char rFRA;
  unsigned char rPRB;
  unsigned char rSPB;
  unsigned char rFRB;
  unsigned char rPRC;
  unsigned char rSPC;
  unsigned char rFRC;
  unsigned char rPRS;
  unsigned char rSPS;
  unsigned char rFRS;
} GripperOutput;

typedef struct __attribute__ ((aligned (8))) _rgi
{
  unsigned char gACT;
  unsigned char gMOD;
  unsigned char gGTO;
  unsigned char gIMC;
  unsigned char gSTA;
  unsigned char gDTA;
  unsigned char gDTB;
  unsigned char gDTC;
  unsigned char gDTS;
  unsigned char gFLT;
  unsigned char gPRA;
  unsigned char gPOA;
  unsigned char gCUA;
  unsigned char gPRB;
  unsigned char gPOB;
  unsigned char gCUB;
  unsigned char gPRC;
  unsigned char gPOC;
  unsigned char gCUC;
  unsigned char gPRS;
  unsigned char gPOS;
  unsigned char gCUS;
} GripperInput;

class R3FGNetwork
{
  public:
    R3FGNetwork(ros::NodeHandle* _rn);
    void waitForNet();
    void connectGripper();
    void sendCommand(irl_robots::r3fgControl com);
    void sendCommand(irl_robots::gSimpleControl com);

    unsigned char getInitializationStatus()          { return p_gripper_input.gACT; }
    unsigned char getOperationModeStatus()           { return p_gripper_input.gMOD; }
    unsigned char getActionStatus()                  { return p_gripper_input.gGTO; }
    unsigned char getGripperStatus()                 { return p_gripper_input.gIMC; }
    unsigned char getMotionStatus()                  { return p_gripper_input.gSTA; }
    unsigned char getFingerAObjectDetectionStatus()  { return p_gripper_input.gDTA; }
    unsigned char getFingerBObjectDetectionStatus()  { return p_gripper_input.gDTB; }
    unsigned char getFingerCObjectDetectionStatus()  { return p_gripper_input.gDTC; }
    unsigned char getScissorObjectDetectionStatus()  { return p_gripper_input.gDTS; }
    unsigned char getFaultStatus()                   { return p_gripper_input.gFLT; }
    unsigned char getFingerARequestedPosition()      { return p_gripper_input.gPRA; }
    unsigned char getFingerBRequestedPosition()      { return p_gripper_input.gPRB; }
    unsigned char getFingerCRequestedPosition()      { return p_gripper_input.gPRC; }
    unsigned char getScissorRequestedPosition()      { return p_gripper_input.gPRS; }
    unsigned char getFingerAPosition()               { return p_gripper_input.gPOA; }
    unsigned char getFingerBPosition()               { return p_gripper_input.gPOB; }
    unsigned char getFingerCPosition()               { return p_gripper_input.gPOC; }
    unsigned char getScissorPosition()               { return p_gripper_input.gPOS; }
    double getFingerACurrentConsumption()            { return p_gripper_input.gCUA * 0.1; }
    double getFingerBCurrentConsumption()            { return p_gripper_input.gCUB * 0.1; }
    double getFingerCCurrentConsumption()            { return p_gripper_input.gCUC * 0.1; }
    double getScissorCurrentConsumption()            { return p_gripper_input.gCUS * 0.1; }

    void setActivateGripper(bool value)              { p_gripper_output.rACT = value?0x01:0x00; }
    void setGraspingMode(unsigned char value)        { p_gripper_output.rMOD = value; }
    void setGoTo(bool value)                         { p_gripper_output.rGTO = value?0x01:0x00; }
    void setAutomaticRelease(bool value)             { p_gripper_output.rATR = value?0x01:0x00; }
    void setGloveMode(bool value)                    { p_gripper_output.rGLV = value?0x01:0x00; }
    void setIndividualFingerControl(bool value)      { p_gripper_output.rICF = value?0x01:0x00; }
    void setIndividualScissorControl(bool value)     { p_gripper_output.rICS = value?0x01:0x00; }
    void setFingerAPosition(unsigned char value)     { p_gripper_output.rPRA = value; }
    void setFingerBPosition(unsigned char value)     { p_gripper_output.rPRB = value; }
    void setFingerCPosition(unsigned char value)     { p_gripper_output.rPRC = value; }
    void setScissorPosition(unsigned char value)     { p_gripper_output.rPRS = value; }
    void setFingerASpeed(unsigned char value)        { p_gripper_output.rSPA = value; }
    void setFingerBSpeed(unsigned char value)        { p_gripper_output.rSPB = value; }
    void setFingerCSpeed(unsigned char value)        { p_gripper_output.rSPC = value; }
    void setScissorSpeed(unsigned char value)        { p_gripper_output.rSPS = value; }
    void setFingerAForce(unsigned char value)        { p_gripper_output.rFRA = value; }
    void setFingerBForce(unsigned char value)        { p_gripper_output.rFRB = value; }
    void setFingerCForce(unsigned char value)        { p_gripper_output.rFRC = value; }
    void setScissorForce(unsigned char value)        { p_gripper_output.rFRS = value; }

  private:
    template <typename type>
    void swap_endian(type& d)
    {
      char& raw = reinterpret_cast<char&>(d);
      std::reverse(&raw, &raw + sizeof(type));
    }

    void netMainLoop();
    void sendNextCommand();
    void readStatus();
    void getMBAPHeader(unsigned char* buffer, unsigned short header_size);
    void getRobotiqHeader(unsigned char* buffer, unsigned char fcode, unsigned short first_reg, unsigned short wordcount);
    void deserializeGripperStatus(unsigned char* data);
    void serializeGripperCommand(unsigned char* data);
    void printData(unsigned char* data, size_t size);
    void writeData(unsigned char* buffer, size_t size);
    void readData(unsigned char*& buffer, size_t size);
    void saveDelete(unsigned char*& buffer);

    std::thread* p_net_thread;
    ros::NodeHandle* p_ros_node;
    ros::Publisher p_pub_status;
    std::string p_gripper_ip;
    int p_gripper_port;
    int p_gripper_refreshrate;
    int p_net_socket;
    struct sockaddr_in p_gripper_socket;
    std::queue<irl_robots::r3fgControl> p_com_queue;
    std::queue<irl_robots::gSimpleControl> p_com_queue2;
    std::mutex p_com_mutex;
    GripperInput p_gripper_input;
    GripperOutput p_gripper_output;
    unsigned short p_packet_id;
    const unsigned char p_packet_clientid = 0x02;
    const unsigned char p_packet_fnset = 0x10;
    const unsigned char p_packet_fnget = 0x04;
};

#endif // R3FGNETWORK_H

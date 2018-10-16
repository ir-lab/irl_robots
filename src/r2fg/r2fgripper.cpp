#include "r2fgripper.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <iomanip>

#include <irl_robots/r2fgStatus.h>

// Interesting stuff/source: http://www.cmrr.umn.edu/~strupp/serial.html

R2FGripper::R2FGripper()
{
  if(!m_ros_node.getParam("r2fg/port", m_serial_port))
  {
    ROS_ERROR("Could not get r2fg/port from parameter server");
    std::exit(1);
  }

  memset(&m_gripper_input, 0, sizeof(GInput));
  memset(&m_gripper_output, 0, sizeof(GOutput));

  m_sub_control  = m_ros_node.subscribe("/r2fg/control", 1, &R2FGripper::callbackControl, this);
  m_sub_control2 = m_ros_node.subscribe("/r2fg/simplecontrol", 1, &R2FGripper::callbackSimpleControl, this);
  m_pub_status   = m_ros_node.advertise<irl_robots::r2fgStatus>("/r2fg/status", 1);
}

R2FGripper::~R2FGripper()
{
  close(m_serial_handle);
}

void R2FGripper::connectGripper()
{
  ROS_INFO("Connecting to gripper at: %s", m_serial_port.c_str());
  m_serial_handle = open(m_serial_port.c_str(), O_RDWR | O_NOCTTY);
  if (m_serial_handle == -1)
  {
    ROS_ERROR("Could not open serial port: %s", m_serial_port.c_str());
    std::exit(1);
  }
  setSerialAttributes();
  ROS_INFO("Connected");

  ROS_INFO("Activating gripper ...");

  setValues(); // reset
  m_gripper_output.rACT = 1;
  setValues();

  ros::Rate loop_rate(10);
  while(ros::ok() && m_gripper_input.gSTA != 3)
  {
    getValues();

    loop_rate.sleep();
  }
  ROS_INFO("... complete");
  ROS_INFO("Entering ROS main loop");

  m_gripper_output.rGTO = 1;
  ros::Rate updaterate(30);
  while(ros::ok())
  {
    ros::spinOnce();
    getValues();
    updaterate.sleep();
  }
}

void R2FGripper::setSerialAttributes()
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if(tcgetattr(m_serial_handle, &tty) != 0)
    {
        throw std::runtime_error("Error with tcgetattr.");
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 5;            // blocking read
    tty.c_cc[VTIME] = 30;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    //tty.c_cflag |= 0; // Parity 8n1 is 0
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if(tcsetattr(m_serial_handle, TCSANOW, &tty) != 0)
    {
        throw std::runtime_error("Error with tcsetattr.");
    }
}

void R2FGripper::callbackControl(irl_robots::r2fgControl c)
{
  m_gripper_output.rACT = c.activate;
  m_gripper_output.rARD = c.autorelease_open;
  m_gripper_output.rATR = c.autorelease;
  m_gripper_output.rFR = c.force;
  m_gripper_output.rGTO = c.go;
  m_gripper_output.rPR = c.position;
  m_gripper_output.rSP = c.speed;
  setValues();
}

void R2FGripper::callbackSimpleControl(irl_robots::gSimpleControl c)
{
  m_gripper_output.rFR = c.force;
  m_gripper_output.rPR = c.position;
  m_gripper_output.rSP = c.speed;
  setValues();
}

unsigned short R2FGripper::computeCRC(unsigned char* adr, unsigned short length)
{
  unsigned short CRC_calc = 0xFFFF;
  int j=0;
  int k=0;

  if (adr == NULL)
  {
    return 0;
  }

  while (j < length)
  {
    if (j==0)
    {
      CRC_calc ^= *adr & 0xFF;
    }
    else
    {
      CRC_calc ^= *adr;
    }

    k=0;

    while (k < 8)
    {
      if (CRC_calc & 0x0001)
      {
        CRC_calc =  (CRC_calc >> 1)^ 0xA001;
      }
      else
      {
        CRC_calc >>= 1;
      }

      k++;
    }

    adr++;
    j++;
  }

  return CRC_calc;
}

void R2FGripper::createCommand(unsigned char* data)
{
  memset(data, 0, sizeof(unsigned char) * 6);

  data[0] = data[0] | (m_gripper_output.rARD << 5);
  data[0] = data[0] | (m_gripper_output.rATR << 4);
  data[0] = data[0] | (m_gripper_output.rGTO << 3);
  data[0] = data[0] | (m_gripper_output.rACT << 0);

  data[3] = m_gripper_output.rPR;
  data[4] = m_gripper_output.rSP;
  data[5] = m_gripper_output.rFR;
}

void R2FGripper::parseState(unsigned char* data)
{
  memset(&m_gripper_input, 0, sizeof(GInput));

  m_gripper_input.gACT = (data[0] >> 0) & 1;
  m_gripper_input.gGTO = (data[0] >> 3) & 1;
  m_gripper_input.gSTA = (data[0] >> 4) & 3;
  m_gripper_input.gOBJ = (data[0] >> 6) & 3;

  m_gripper_input.gFLT = (data[2] >> 0) & 15;
  m_gripper_input.kFLT = (data[2] >> 4) & 15;

  m_gripper_input.gPR = data[3];
  m_gripper_input.gPO = data[4];
  m_gripper_input.gCU = data[5];

  irl_robots::r2fgStatus msg;
  msg.activationstatus = m_gripper_input.gACT;
  msg.actionstatus = m_gripper_input.gGTO;
  msg.objectstatus = m_gripper_input.gOBJ;
  msg.gripperstatus = m_gripper_input.gSTA;
  msg.fault_sw = m_gripper_input.gFLT;
  msg.fault_hw = m_gripper_input.kFLT;
  msg.position_echo = m_gripper_input.gPR;
  msg.position = m_gripper_input.gPO;
  msg.current = m_gripper_input.gCU * 10.0;
  m_pub_status.publish(msg);
}

void R2FGripper::getValues()
{
//  ROS_INFO("Getting values");
  unsigned char* buffer = new unsigned char[8];
  buffer[0] = (unsigned char)SLAVE_ID;
  buffer[1] = (unsigned char)READ_HOLDING_REGISTERS;
  buffer[2] = (unsigned char)0x07; // First address byte
  buffer[3] = (unsigned char)0xD0; // Second address byte
  buffer[4] = (unsigned char)0x00; // First number of requested registers
  buffer[5] = (unsigned char)0x03; // Second number of requested registers

  unsigned short crc = computeCRC(buffer, 6);
  memcpy(buffer + 6, &crc, 2);

  serialWrite(buffer, 8);

  serialRead(&buffer, 11);

  if(!buffer)
    return;

  if(buffer[0] == 0x09) // Make sure we really got valid data
  {
    parseState(buffer+3);

    //    emit this->signal_updatedGripperState(input.gOBJ, input.gPO, input.gCU, input.gFLT);
    if(m_gripper_input.gFLT != 0 && m_gripper_input.gFLT != 9)
    {
      ROS_INFO("Gripper Error with ID: %d", m_gripper_input.gFLT);
    }
  }
  delete buffer;
}

void R2FGripper::setValues()
{
//  ROS_INFO("Setting Values");
  unsigned char* buffer = new unsigned char[15];
  buffer[0] = (unsigned char)SLAVE_ID;
  buffer[1] = (unsigned char)PRESET_MULTIPLE_REGISTERS;
  buffer[2] = (unsigned char)0x03; // First Register part 1
  buffer[3] = (unsigned char)0xE8; // First Register part 2
  buffer[4] = (unsigned char)0x00; // Number of registers to write part 1
  buffer[5] = (unsigned char)0x03; // Number of registers to write part 2
  buffer[6] = (unsigned char)0x06; // Number of bytes to follow (payload)

  createCommand(buffer + 7);

  unsigned short crc = computeCRC(buffer, 13);
  memcpy(buffer + 13, &crc, 2);

  serialWrite(buffer, 15);

  serialRead(&buffer, 8); // Discard confirmation...
  if(buffer)
    delete buffer;
}

void R2FGripper::printData(unsigned char* data, size_t size)
{
  // debug
  std::ios::fmtflags f(std::cout.flags() );
  for(auto i = 0; i < size; i++)
    std::cout << " " << std::setw(2) << std::setfill('0') << std::hex << (int)data[i];
  std::cout << std::endl;
  std::cout.flags( f );
}

void R2FGripper::serialWrite(unsigned char* data, size_t size)
{
//  std::cout << "Writing: ";
//  printData(data, size);
  for(auto i = 0; i < size; )
  {
    size_t n = write(m_serial_handle, data + i, size - i);
    if(n < 0)
    {
      ROS_ERROR("Communication Error: Failed to write bytes");
      std::exit(1);
    }
    i += n;
  }
}

void R2FGripper::serialRead(unsigned char** buffer, unsigned short size)
{
  if(*buffer)
    delete *buffer;
  *buffer = new unsigned char[size];

  for(auto i = 0; i < size; )
  {
    ssize_t n = read(m_serial_handle, (*buffer) + i, size - i);
    if (n < 0)
    {
      ROS_ERROR("Communication Error: Failed to read bytes: %s", strerror(errno));
      std::exit(1);
    }
    i += n;
  }

  unsigned short crc_p;
  memcpy((char*)&crc_p, *buffer + size - 2, 2);
  if(computeCRC(*buffer, size - 2) != crc_p)
  {
    ROS_INFO("CRC missmatch in received package");
    delete *buffer;
    *buffer = 0;
  }

//  std::cout << "Receiving: ";
//  printData(*buffer, size);
}




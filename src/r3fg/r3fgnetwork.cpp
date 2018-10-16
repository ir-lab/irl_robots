#include "r3fgnetwork.h"

#include <irl_robots/r3fgStatus.h>

#include <sys/socket.h>
#include <cstring>
#include <netdb.h>

R3FGNetwork::R3FGNetwork(ros::NodeHandle* _rn) :
  p_ros_node(_rn),
  p_net_socket(-1),
  p_packet_id(0)
{
  p_pub_status  = p_ros_node->advertise<irl_robots::r3fgStatus>("/r3fg/status", 1);

  if(!p_ros_node->getParam("r3fg/ip", p_gripper_ip))
    ROS_ERROR("Could not get r3fg/ip from parameter server");
  if(!p_ros_node->getParam("r3fg/port", p_gripper_port))
    ROS_ERROR("Could not get r3fg/port from parameter server");
  if(!p_ros_node->getParam("r3fg/refreshrate", p_gripper_refreshrate))
  {
    ROS_ERROR("Could not get r3fg/refreshrate from parameter server");
    std::exit(1);
  }

  connectGripper();

  memset(&p_gripper_input, 0, sizeof(GripperInput));
  memset(&p_gripper_output, 0, sizeof(GripperOutput));

  p_net_thread = new std::thread(&R3FGNetwork::netMainLoop, this);

  setActivateGripper(true);
}

void R3FGNetwork::waitForNet()
{
  p_net_thread->join();
}

void R3FGNetwork::connectGripper()
{
  ROS_INFO("Connecting to Gripper at %s:%d", p_gripper_ip.c_str(), p_gripper_port);

  if(p_net_socket == -1)
  {
    p_net_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(p_net_socket == -1)
    {
      ROS_ERROR("Can't create Gripper socket");
      std::exit(1);
    }
  }

  struct hostent*  host;
  struct in_addr** address;
  host = gethostbyname(p_gripper_ip.c_str());

  if(host == NULL)
  {
    ROS_ERROR("Can't resolve host %s", p_gripper_ip.c_str());
    std::exit(1);
  }

  address = (struct in_addr**)host->h_addr_list;
  ROS_INFO("%s resolved to %s: trying to connect", p_gripper_ip.c_str(), inet_ntoa(*address[0]));

  p_gripper_socket.sin_addr = *address[0];
  p_gripper_socket.sin_family = AF_INET;
  p_gripper_socket.sin_port = htons(p_gripper_port);

  if (connect(p_net_socket, (struct sockaddr *)&p_gripper_socket , sizeof(struct sockaddr_in)) < 0)
  {
    ROS_ERROR("Can't connect to host %s", p_gripper_ip.c_str());
    std::exit(1);
  }

  ROS_INFO("Connected to Gripper");
}

void R3FGNetwork::sendCommand(irl_robots::r3fgControl com)
{
  std::lock_guard<std::mutex> lock(p_com_mutex);
  p_com_queue.push(com);
}

void R3FGNetwork::sendCommand(irl_robots::gSimpleControl com)
{
  std::lock_guard<std::mutex> lock(p_com_mutex);
  p_com_queue2.push(com);
}

void R3FGNetwork::netMainLoop()
{
  ROS_INFO("Launching Network main loop");

  ros::Rate loop_rate(p_gripper_refreshrate);
  while(ros::ok())
  {
    sendNextCommand();
    readStatus();
    loop_rate.sleep();
  }
}

void R3FGNetwork::sendNextCommand()
{
  std::lock_guard<std::mutex> lock(p_com_mutex);
  bool run = false;

  if(p_com_queue.size() > 0)
  {
    run = true;
    setActivateGripper(p_com_queue.front().activate);
    setGraspingMode(p_com_queue.front().mode);
    setGoTo(p_com_queue.front().go);
    setAutomaticRelease(p_com_queue.front().autorelease);
    setGloveMode(p_com_queue.front().glove);
    setIndividualFingerControl(p_com_queue.front().individualfinger);
    setIndividualScissorControl(p_com_queue.front().individualscissor);
    setFingerAForce(p_com_queue.front().finger[0].force);
    setFingerAPosition(p_com_queue.front().finger[0].position);
    setFingerASpeed(p_com_queue.front().finger[0].speed);
    setFingerBForce(p_com_queue.front().finger[1].force);
    setFingerBPosition(p_com_queue.front().finger[1].position);
    setFingerBSpeed(p_com_queue.front().finger[1].speed);
    setFingerCForce(p_com_queue.front().finger[2].force);
    setFingerCPosition(p_com_queue.front().finger[2].position);
    setFingerCSpeed(p_com_queue.front().finger[2].speed);
    setScissorForce(p_com_queue.front().scissor.force);
    setScissorPosition(p_com_queue.front().scissor.position);
    setScissorSpeed(p_com_queue.front().scissor.speed);

    p_com_queue.pop();
  }
  else if(p_com_queue2.size() > 0)
  {
    run = true;
    setGoTo(true);
    setActivateGripper(true);
    setGraspingMode(0);
    setFingerAForce(p_com_queue2.front().force);
    setFingerAPosition(p_com_queue2.front().position);
    setFingerASpeed(p_com_queue2.front().speed);

    p_com_queue2.pop();
  }
  if(run)
  {
    unsigned char* data = new unsigned char[16];
    unsigned char* buffer = new unsigned char[16 + 7 + 5 + 1]; // Data + MBAPHeader + RobotiqHeader + size
    serializeGripperCommand(data);
    getMBAPHeader(buffer, 16 + 1); // + 1 for "bytes to follow" byte
    getRobotiqHeader(buffer + 7, p_packet_fnset, 0x0000, 0x0008);
    unsigned short ds = 16;
    memcpy(buffer + 12, &ds, 1);
    memcpy(buffer + 13, data, 16);

    writeData(buffer, 29);
    readData(buffer, 12);


    saveDelete(data);
    saveDelete(buffer);
  }
}

void R3FGNetwork::readStatus()
{
  unsigned char* buffer = new unsigned char[7 + 5]; // MBAPHeader + RobotiqHeader
  getMBAPHeader(buffer, 0); // + 1 for "bytes to follow" byte
  getRobotiqHeader(buffer + 7, p_packet_fnget, 0x0000, 0x0008);
  writeData(buffer, 12);
  saveDelete(buffer);

  readData(buffer, 25);

  deserializeGripperStatus(buffer + 9);
  irl_robots::r3fgStatus msg;
  msg.activationstatus =            getInitializationStatus();
  msg.operationstatus =             getOperationModeStatus();
  msg.actionstatus =                getActionStatus();
  msg.gripperstatus =               getGripperStatus();
  msg.motionstatus =                getMotionStatus();
  msg.faultstatus =                 getFaultStatus();
  msg.finger[0].motionstatus =      getFingerAObjectDetectionStatus();
  msg.finger[0].positionrequested = getFingerARequestedPosition();
  msg.finger[0].positionactual =    getFingerAPosition();
  msg.finger[0].current =           getFingerACurrentConsumption();
  msg.finger[1].motionstatus =      getFingerBObjectDetectionStatus();
  msg.finger[1].positionrequested = getFingerBRequestedPosition();
  msg.finger[1].positionactual =    getFingerBPosition();
  msg.finger[1].current =           getFingerBCurrentConsumption();
  msg.finger[2].motionstatus =      getFingerCObjectDetectionStatus();
  msg.finger[2].positionrequested = getFingerCRequestedPosition();
  msg.finger[2].positionactual =    getFingerCPosition();
  msg.finger[2].current =           getFingerCCurrentConsumption();
  msg.scissor.motionstatus =        getScissorObjectDetectionStatus();
  msg.scissor.positionrequested =   getScissorRequestedPosition();
  msg.scissor.positionactual =      getScissorPosition();
  msg.scissor.current =             getScissorCurrentConsumption();
  p_pub_status.publish(msg);
  saveDelete(buffer);
}

void R3FGNetwork::getMBAPHeader(unsigned char* buffer, unsigned short header_size)
{
  // buffer needs to be pre-initialized to have at least a size of 7
  header_size += 1 + 5; // Gripper data (size) + clientidentifier (1) + Robotiq header (5)
  swap_endian<unsigned short>(header_size);

  unsigned short tid = p_packet_id;
  swap_endian<unsigned short>(tid);

  memcpy(buffer, &tid, sizeof(unsigned short));
  memset(buffer + 2, 0, sizeof(unsigned short));
  memcpy(buffer + 4, &header_size, sizeof(unsigned short));
  memcpy(buffer + 6, &p_packet_clientid, sizeof(unsigned char));

  p_packet_id++;
}

void R3FGNetwork::getRobotiqHeader(unsigned char* buffer, unsigned char fcode, unsigned short first_reg, unsigned short wordcount)
{
  swap_endian<unsigned char>(fcode);
  swap_endian<unsigned short>(first_reg);
  swap_endian<unsigned short>(wordcount);

  memcpy(buffer + 0, &fcode, sizeof(unsigned char));
  memcpy(buffer + 1, &first_reg, sizeof(unsigned short));
  memcpy(buffer + 3, &wordcount, sizeof(unsigned short));
}

void R3FGNetwork::deserializeGripperStatus(unsigned char* data)
{
  // Gripper Status
  p_gripper_input.gACT = data[0] & 0x1;
  p_gripper_input.gMOD = (data[0] >> 0x1) & 0x3;
  p_gripper_input.gGTO = (data[0] >> 0x3) & 0x1;
  p_gripper_input.gIMC = (data[0] >> 0x4) & 0x3;
  p_gripper_input.gSTA = (data[0] >> 0x6) & 0x3;

  // Object Status
  p_gripper_input.gDTA = data[1] & 0x3;
  p_gripper_input.gDTB = (data[1] >> 0x2) & 0x3;
  p_gripper_input.gDTC = (data[1] >> 0x4) & 0x3;
  p_gripper_input.gDTS = (data[1] >> 0x6) & 0x3;

  // Fault Status
  p_gripper_input.gFLT = data[2] & 0xF;

  // Requested Position, Speed and Force (Finger A).
  p_gripper_input.gPRA = data[3];
  p_gripper_input.gPOA = data[4];
  p_gripper_input.gCUA = data[5];

  // Finger B
  p_gripper_input.gPRB = data[6];
  p_gripper_input.gPOB = data[7];
  p_gripper_input.gCUB = data[8];

  // Finger C
  p_gripper_input.gPRC = data[9];
  p_gripper_input.gPOC = data[10];
  p_gripper_input.gCUC = data[11];

  // Scissor Mode
  p_gripper_input.gPRS = data[12];
  p_gripper_input.gPOS = data[13];
  p_gripper_input.gCUS = data[14];
}

void R3FGNetwork::serializeGripperCommand(unsigned char* data)
{
  // Pack the Action Request register byte
  data[0] = (p_gripper_output.rACT & 0x1) | (p_gripper_output.rMOD << 0x1) & 0x6 | ((p_gripper_output.rGTO << 0x3) & 0x8) | ((p_gripper_output.rATR << 0x4) & 0x10);

  // Pack the Gripper Options register byte
  data[1] =  ((p_gripper_output.rICF << 0x2) & 0x4) | ((p_gripper_output.rICS << 0x3) & 0x8);

  // map[2] is empty

  // Requested Position, Speed and Force (Finger A).
  data[3]  = p_gripper_output.rPRA;
  data[4]  = p_gripper_output.rSPA;
  data[5]  = p_gripper_output.rFRA;

  // Finger B
  data[6]  = p_gripper_output.rPRB;
  data[7]  = p_gripper_output.rSPB;
  data[8]  = p_gripper_output.rFRB;

  // Finger C
  data[9]  = p_gripper_output.rPRC;
  data[10] = p_gripper_output.rSPC;
  data[11] = p_gripper_output.rFRC;

  // Scissor Mode
  data[12] = p_gripper_output.rPRS;
  data[13] = p_gripper_output.rSPS;
  data[14] = p_gripper_output.rFRS;

  data[15] = 0x00; // padding to get 8 words
}

void R3FGNetwork::printData(unsigned char* data, size_t size)
{
  // debug
  std::ios::fmtflags f(std::cout.flags() );
  for(auto i = 0; i < size; i++)
    std::cout << " " << std::setw(2) << std::setfill('0') << std::hex << (int)data[i];
  std::cout << std::endl;
  std::cout.flags( f );
}

void R3FGNetwork::writeData(unsigned char* buffer, size_t size)
{
  for(auto i = 0; i < size;)
  {
    int sds = send(p_net_socket, buffer + i, size - i, 0);

    if(sds < 0)
    {
      ROS_ERROR("Communication error with gripper");
      std::exit(1);
    }
    i += sds;
  }
}

void R3FGNetwork::readData(unsigned char*& buffer, size_t size)
{
  if(buffer)
    saveDelete(buffer);
  buffer = new unsigned char[size];
  for(auto i = 0; i < size;)
  {
    int rvs = recv(p_net_socket, buffer + i, size - i, 0);
    if(rvs < 0)
    {
      ROS_ERROR("Communication error with gripper");
      std::exit(1);
    }
    else if(rvs == 0)
    {
      ROS_INFO("Gripper closed communication. Shutting down.");
      std::exit(0);
    }
    i += rvs;
  }
}

void R3FGNetwork::saveDelete(unsigned char*& buffer)
{
  if(buffer)
    delete buffer;
  buffer = 0;
}

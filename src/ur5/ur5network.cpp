#include "ur5network.h"

#include <sys/socket.h>
#include <netdb.h>
#include <cstring>

#include <irl_robots/ur5Tool.h>
#include <irl_robots/ur5Status.h>

UR5Network::UR5Network(ros::NodeHandle* n) :
  p_ros_node(n),
  p_net_socket(-1),
  p_integral_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  p_differential_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
    XmlRpc::XmlRpcValue param_list;
    if(n->getParam("control", param_list))
    {
        m_p_gain = param_list["ur5"]["p_gain"];
        m_i_gain = param_list["ur5"]["i_gain"];
        m_d_gain = param_list["ur5"]["d_gain"];
        m_max_velocity = param_list["ur5"]["max_velocity"];
        m_i_max = param_list["ur5"]["max_i"];
        m_i_min = param_list["ur5"]["min_i"];
        m_max_accel = param_list["ur5"]["max_acceleration"];
    }
    else
    {
        throw std::invalid_argument("Missing \"control\" ROS parameters for UR5. Please set them and re-run.");
    }

  p_pub_joint  = p_ros_node->advertise<irl_robots::ur5Joints>("/ur5/joints", 1);
  p_pub_tool   = p_ros_node->advertise<irl_robots::ur5Tool>  ("/ur5/tool",   1);
  p_pub_status = p_ros_node->advertise<irl_robots::ur5Status>("/ur5/status", 1);

  if(!p_ros_node->getParam("ur5/ip", p_ur5_ip))
    ROS_INFO("Could not get ur5/ip from parameter server");
  if(!p_ros_node->getParam("ur5/port", p_ur5_port))
    ROS_INFO("Could not get ur5/port from parameter server");

  connectUR5();

  p_net_thread = new std::thread(&UR5Network::netMainLoop, this);
}

void UR5Network::waitForNet()
{
  p_net_thread->join();
}

void UR5Network::sendCommand(irl_robots::ur5Control com)
{
  std::lock_guard<std::mutex> lock(p_com_mutex);
  p_com_queue.push(com);
}

void UR5Network::netMainLoop()
{
  ROS_INFO("Launching Network main loop");

  while(ros::ok())
  {
    sendNextCommand();
    readStatus();
  }
}

void UR5Network::connectUR5()
{
  ROS_INFO("Connecting to UR5 at %s:%d", p_ur5_ip.c_str(), p_ur5_port);

  if(p_net_socket == -1)
  {
    p_net_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(p_net_socket == -1)
    {
      ROS_ERROR("Can't create UR5 socket");
      std::exit(1);
    }
  }

  struct hostent*  host;
  struct in_addr** address;
  host = gethostbyname(p_ur5_ip.c_str());

  if(host == NULL)
  {
    ROS_ERROR("Can't resolve host %s", p_ur5_ip.c_str());
    std::exit(1);
  }

  address = (struct in_addr**)host->h_addr_list;
  ROS_INFO("%s resolved to %s: trying to connect", p_ur5_ip.c_str(), inet_ntoa(*address[0]));

  p_ur5_socket.sin_addr = *address[0];
  p_ur5_socket.sin_family = AF_INET;
  p_ur5_socket.sin_port = htons(p_ur5_port);

  if (connect(p_net_socket, (struct sockaddr *)&p_ur5_socket , sizeof(struct sockaddr_in)) < 0)
  {
    ROS_ERROR("Can't connect to host %s", p_ur5_ip.c_str());
    std::exit(1);
  }

  ROS_INFO("Connected to UR5");
}

void UR5Network::sendNextCommand()
{
  std::lock_guard<std::mutex> lock(p_com_mutex);

  if(p_com_queue.size() == 0)
    return;

  if(p_ur5_mode != 7.0)
  {
    ROS_ERROR("UR5 is not in running mode. Discarding commands.");
    p_com_queue.pop();
    return;
  }

  std::string command = "";
  command += p_com_queue.front().command;
  command += "(";

  if(!p_com_queue.front().jointcontrol)
    command += "p";

  command += "[";

  //std::cout << "Velocity ";

  for(auto i = 0; i < 6; i ++)
  {
    if(p_com_queue.front().command == "speedj")
    {
      double error = p_com_queue.front().values[i] - p_ur5_currjoints.positions[i];

      p_integral_state[i] += error;

      // Min/max bounds prevent integral windup.
      if(p_integral_state[i] > m_i_max)
      {
          p_integral_state[i] = m_i_max;
      }
      else if(p_integral_state[i] < m_i_min)
      {
          p_integral_state[i] = m_i_min;
      }

      double velocity =
        (error * m_p_gain) +
        (p_integral_state[i] * m_i_gain) +
        ((error - p_differential_state[i]) * m_d_gain);

      p_differential_state[i] = error;

      if(velocity > 0.0)
      {
        velocity = std::min(m_max_velocity, velocity);
      }
      else
      {
        velocity = std::max(-m_max_velocity, velocity);
      }

      //std::cout << "| " << velocity;

      command += std::to_string(velocity);
//      std::cout << i << " " << actual_velocity << " " << (p_com_queue.front().values[i] - p_ur5_currjoints.positions[i]) / (double)floor(1.0 / p_com_queue.front().time) << " | ";
      //      command += std::to_string((p_com_queue.front().values[i] - p_ur5_currjoints.positions[i]) / (double)floor(1.0 / p_com_queue.front().time)); // carrot
    }
    else
      command += std::to_string(p_com_queue.front().values[i]);
    if(i < 5)
      command += ",";
  }
  //std::cout << std::endl;

  command += "]";
  if(p_com_queue.front().command == "movel" || p_com_queue.front().command == "movej")
  {
    if(p_com_queue.front().acceleration > 0.0) command += ",a=" + std::to_string(p_com_queue.front().acceleration);
    if(p_com_queue.front().velocity > 0.0)     command += ",v=" + std::to_string(p_com_queue.front().velocity);
    if(p_com_queue.front().time > 0.0)         command += ",t=" + std::to_string(p_com_queue.front().time);
    if(p_com_queue.front().blend > 0.0)        command += ",r=" + std::to_string(p_com_queue.front().blend);
  }
  else if(p_com_queue.front().command == "servoj")
  {
    command += ",t=" + std::to_string(p_com_queue.front().time);
    command += ",lookahead_time=" + std::to_string(p_com_queue.front().acceleration);
    command += ",gain=" + std::to_string(p_com_queue.front().velocity);
  }
  else if(p_com_queue.front().command == "speedj")
  {
    p_com_queue.front().acceleration = m_max_accel;
    command += "," + std::to_string(p_com_queue.front().acceleration);
    command += "," + std::to_string(p_com_queue.front().time);
  }
  else
  {
    ROS_ERROR("Command %s is not implemented", p_com_queue.front().command.c_str());
    p_com_queue.pop();
    return;
  }
  command += ")\n";

  p_com_queue.pop();

  for(auto i = 0; i < strlen(command.c_str());)
  {
    int sds = send(p_net_socket, command.c_str() + i, strlen(command.c_str()) - i, 0);

    if(sds < 0)
    {
      ROS_ERROR("Communication error with UR5");
      std::exit(1);
    }
    i += sds;
  }
}

void UR5Network::readStatus()
{
  char buffer[1056];
  int size = 0;
  for(auto i = 0; i < sizeof(int);)
  {
    int rvs = recv(p_net_socket, ((char*)(&size)) + i, sizeof(int) - i, 0);
    if(rvs < 0)
    {
      ROS_ERROR("Communication error with UR5");
      std::exit(1);
    }
    else if(rvs == 0)
    {
      ROS_INFO("UR5 closed communication. Shutting down.");
      std::exit(0);
    }

    i += rvs;
  }

  swapByteorder<int>(&size);

  if(size != 1060)
  {
    ROS_ERROR("UR5 state size is %d, but 1060 expected.", size);
    std::exit(1);
  }

  for(auto i = 0; i < 1056;)
  {
    int rvs = recv(p_net_socket, buffer + i, 1056 - i, 0);
    if(rvs < 0)
    {
      ROS_ERROR("Communication error with UR5");
      std::exit(1);
    }
    else if(rvs == 0)
    {
      ROS_INFO("UR5 closed communication. Shutting down.");
      std::exit(0);
    }
    else
      i += rvs;
  }

  parseStatus(buffer);
}

void UR5Network::parseStatus(char* buffer)
{
  irl_robots::ur5Joints jmsg;
  irl_robots::ur5Tool tmsg;
  irl_robots::ur5Status smsg;

  memcpy(jmsg.positions.data(),  buffer + 31 * sizeof(double), 6 * sizeof(double));
  memcpy(jmsg.velocities.data(), buffer + 37 * sizeof(double), 6 * sizeof(double));
  memcpy(tmsg.position.data(),   buffer + 55 * sizeof(double), 3 * sizeof(double));
  memcpy(tmsg.rotation.data(),   buffer + 58 * sizeof(double), 3 * sizeof(double));
  memcpy(tmsg.velocities.data(), buffer + 61 * sizeof(double), 6 * sizeof(double));
  memcpy((void*)&smsg.mode,      buffer + 94 * sizeof(double), 1 * sizeof(double));

  for(auto i = 0; i < 6; i++)
  {
    swapByteorder<double>(&jmsg.positions[i]);
    swapByteorder<double>(&jmsg.velocities[i]);
    swapByteorder<double>(&tmsg.velocities[i]);
    if(i < 3)
    {
      swapByteorder<double>(&tmsg.position[i]);
      swapByteorder<double>(&tmsg.rotation[i]);
    }
  }
  swapByteorder<double>(&smsg.mode);

  p_pub_status.publish(smsg);
  p_pub_joint.publish(jmsg);
  p_pub_tool.publish(tmsg);

  p_ur5_mode = smsg.mode;
  p_ur5_currjoints = jmsg;
}

#include "robotont_driver/hardware.h"

namespace robotont
{
Hardware::Hardware() : reconnect_requested_(false)
{
  ROS_DEBUG("Loading hardware communication class ...");

  // Get parameters from ROS parameter server
  std::string robotont_port;
  int robotont_baudrate;

  nh_.param("serial/port", robotont_port, std::string("/dev/ttyACM0"));
  nh_.param("serial/baudrate", robotont_baudrate, 115200);

  // Configure serial
  serial_.setPort(robotont_port);
  serial_.setBaudrate(robotont_baudrate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(timeout);

  connect();
  
  // Set up a timer to reconnect whenever the connection to hardware should drop
  timer_ = nh_.createTimer(ros::Duration(.5), &Hardware::checkConnection, this);
}

Hardware::~Hardware()
{
  // Send ESC key to stop the motors on exit
  RobotontPacket packet;
  packet.push_back("\x1B");
  writePacket(packet);
}

void Hardware::connect()
{
  if (serial_.isOpen())
  {
    // Connection already open, close it before reconnecting
    serial_.close();
  }

  // Try to open the serial port
  try
  {
    serial_.open();
  }
  catch (serial::IOException e)
  {
    ROS_ERROR_STREAM_THROTTLE(1, "Unable to open port '" << serial_.getPort() << "': " << e.what());
  }
  catch (serial::SerialException e)
  {
    ROS_ERROR_STREAM_THROTTLE(1, "Unable to open port '" << serial_.getPort() << "': " << e.what());
  }

  if (serial_.isOpen())
  {
    ROS_DEBUG_STREAM("Connected to serial port '" << serial_.getPort() << "'");
  }
}

void Hardware::checkConnection(const ros::TimerEvent& event)
{
  if (!serial_.isOpen() || reconnect_requested_)
  {
    connect();  // Reconnent if not connected for some reason.
  }
}

bool Hardware::readPacket(RobotontPacket& packet)
{
  // Collect all bytes from the serial buffer.
  try
  {
    size_t bytes_available = serial_.available();
    //  ROS_DEBUG("bytes available: %lu", bytes_available);
    if (bytes_available)
    {
      std::string new_data = "";
      serial_.read(new_data, bytes_available);
      packet_buffer_.append(new_data);  // TODO: no max size set, handle the situation if polled too slowly
    }
  }
  catch (serial::IOException e)
  {
    reconnect_requested_ = true;
  }
  catch (serial::SerialException e)
  {
    reconnect_requested_ = true;
  }
  catch (serial::PortNotOpenedException)
  {
    reconnect_requested_ = true;
  }

  // Trim line endings from the left
  size_t packet_beg_pos = packet_buffer_.find_first_not_of("\r\n");
  if (packet_beg_pos == std::string::npos)
  {
    // buffer empty or contains only newline chars, clear everything
    packet_buffer_ = "";
    return false;
  }
  else
  {
    packet_buffer_.erase(0, packet_beg_pos);  // Trim
  }

  // Analyze the buffer by searching the line ending characters
  size_t packet_end_pos = packet_buffer_.find_first_of("\r\n");
  if (packet_end_pos == std::string::npos)
  {
    // Packet in buffer not yet complete
    return false;
  }

  // The buffer contains at least one symbol marking packet ending
  if (packet_end_pos > 2)  // Check a minimum size requirement for a valid packet
  {
    // Remove this packet from the buffer, the remaining newlines will be trimmed with next call
    std::string packet_str = packet_buffer_.substr(0, packet_end_pos);
    packet_buffer_.erase(0, packet_end_pos);

    // Unflatten the packet
    std::stringstream packet_ss(packet_str);
    std::string arg;
    packet.clear();
    while (std::getline(packet_ss, arg, ':'))
    {
      packet.push_back(arg);
    }
    return true;
  }

  // Invalid packet, clear the buffer
  packet_buffer_ = "";
  return false;
}

void Hardware::writePacket(const RobotontPacket& packet)
{
  if (packet.empty())
  {
    return;
  }

  // Flatten the packet
  std::stringstream packet_ss;
  for (RobotontPacket::const_iterator it = packet.begin(); it != packet.end(); it++)
  {
    if (it + 1 == packet.end())
    {
      packet_ss << *it;
    }
    else
    {
      packet_ss << *it << ":";
    }
  }
  packet_ss << "\r\n";

  try
  {
    ROS_DEBUG_STREAM("Writing '" << packet_ss.str() << "'");
    serial_.write(packet_ss.str());
  }
  catch (serial::IOException e)
  {
    reconnect_requested_ = true;
  }
  catch (serial::SerialException e)
  {
    reconnect_requested_ = true;
  }
  catch (serial::PortNotOpenedException)
  {
    reconnect_requested_ = true;
  }
}
}  // namespace robotont

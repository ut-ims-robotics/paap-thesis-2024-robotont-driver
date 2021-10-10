/* 
* This node communicates with Robotont hardware, subscribes to /cmd_vel and publishes /odom
*/
//#include <rclcpp/rclcpp.hpp>
#include "robotont_driver/robotont_driver.hpp"

using namespace std::chrono_literals;

RobotontDriver::RobotontDriver() : Node("robotont_driver_node"), odom_(nullptr)
{
  RCLCPP_INFO(this->get_logger(), "Robotont driver is starting...");

  std::string robotont_port = "/dev/ttyACM0";
  int robotont_baudrate = 115200;

  // configure serial
  serial_.setPort(robotont_port);
  serial_.setBaudrate(robotont_baudrate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(timeout);

  connect();
  //Port is open, ready to communicate or interrupt occured
  
  // Subscribe to command velocity topic
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), std::bind(&RobotontDriver::cmd_vel_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(1s, std::bind(&RobotontDriver::read, this) );
}

void RobotontDriver::connect()
{
  RCLCPP_INFO(this->get_logger(), "Connecting to %s", serial_.getPort().c_str());

  if (serial_.isOpen())
  {
    // Connection already open, close it before reconnecting
    serial_.close();
  }

  do
  {
    // Try to open the serial port
    try
    {
      serial_.open();
    }
    catch(serial::IOException e)
    {
      std::stringstream ss;
      ss << "Unable to open port '" << serial_.getPort() << "': " << e.what();
      RCLCPP_ERROR(this->get_logger(),ss.str().c_str());
    }
    catch(serial::SerialException e)
    {
      std::stringstream ss;
      ss << "Unable to open port '" << serial_.getPort() << "': " << e.what();
    }

    if (serial_.isOpen())
    {
      RCLCPP_INFO(this->get_logger(), "Serial port is open");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Failed to open Serial port, retrying after 1 sec...");
      // returns immediately in case of an interrupt
      rclcpp::sleep_for(1s);
    }
  }
  while(!serial_.isOpen() && rclcpp::ok());
}


RobotontDriver::~RobotontDriver()
{
  RCLCPP_INFO(this->get_logger(), "Robotont driver is shutting down...");
  std::string packet = "\x1B";
  write(packet);
}

void RobotontDriver::initialize()
{
  try
  {
  odom_ = std::make_unique<Odom>(this->shared_from_this());
//  RCLCPP_INFO(this->get_logger(), "Initialize(): After constructing odom");
  }
  catch (DriverException e)
  {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
}

void RobotontDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  RCLCPP_INFO(this->get_logger(), "got vel cmd");
  writeRobotSpeed(cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);
}

void RobotontDriver::writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z)
{
  std::stringstream ss;
  ss << "RS:";
  ss << lin_vel_x << ":";
  ss << lin_vel_y << ":";
  ss << ang_vel_z << "\r\n";
  write(ss.str());
}

void RobotontDriver::write(const std::string& packet)
{
  try
  {
    serial_.write(packet);
  }
  catch(serial::IOException e)
  {
    connect();
  }
  catch(serial::SerialException e)
  {
    // something went wrong, make sure we're connected
    // TODO: maybe not the best style to reconnect here as this will block the callback until reconnected
    // (same pattern in read())
    connect();
  }
}

void RobotontDriver::read()
{

  std::string buffer = "";
  try
  {
    size_t bytes_available = serial_.available();
    //    RCLCPP_INFO(this->get_logger(), "read(): bytes available %u",serial_.available());

    if (!bytes_available)
    {
      return;
    }

    serial_.read(buffer, bytes_available);
  }
  catch(serial::IOException e)
  {
    connect();
  }
  catch(serial::SerialException e)
  {
    connect();
  }

  while(buffer.size())
  {

    if(buffer[0] == '\r' || buffer[0] == '\n')
    {
      processPacket();
      packet_ = "";
    }
    else
    {
      packet_.push_back(buffer[0]);
    }

    buffer.erase(buffer.begin());
  }
}

void RobotontDriver::processPacket()
{
  if(packet_.length() <= 2)
  {
    return;
  }

  std::stringstream ss(packet_);
  std::string arg[7];
  std::getline(ss, arg[0], ':');

  // parse motor speeds packet format [SPEED:speed_m0:speed_m1:speed_m2]
  if (arg[0] == "ODOM")
  {
    for (int i = 1; i < 7; i++)
    {
      std::getline(ss, arg[i], ':');
      if (!arg[i].length())
      {
        return;  // invalid packet
      }
    }

    float pos_x = atof(arg[1].c_str());
    float pos_y = atof(arg[2].c_str());
    float ori_z = atof(arg[3].c_str());
    float lin_vel_x = atof(arg[4].c_str());
    float lin_vel_y = atof(arg[5].c_str());
    float ang_vel_z = atof(arg[6].c_str());

    RCLCPP_INFO(this->get_logger(), "got ODOM %f : %f : %f", pos_x, pos_y, ori_z);

    if(odom_ != nullptr)
    {
      odom_->update(pos_x, pos_y, ori_z, lin_vel_x, lin_vel_y, ang_vel_z);
      odom_->publish();
    }
  }
}


int main(int argc, char* argv[])
{
  // Initialise ROS2
  rclcpp::init(argc, argv);

  // Create a single threaded executor
  rclcpp::executors::SingleThreadedExecutor exec;

  // Create driver and odom nodes
  auto driver_node = std::make_shared<RobotontDriver>();
  driver_node->initialize();

  // Add nodes to the executor
  exec.add_node(driver_node);

  // Spin
  exec.spin();

  // Do a nice shut down for ROS2
  rclcpp::shutdown();
  return 0;
}

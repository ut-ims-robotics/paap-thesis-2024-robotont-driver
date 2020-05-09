#ifndef ROBOTONT_HARDWARE_
#define ROBOTONT_HARDWARE_

#include <ros/ros.h>
#include <serial/serial.h>

// We have 18 RGB leds, which requires
// 3*18 arguments + 3 for the header
#define MAX_ARGS 60

namespace robotont
{
typedef std::vector<std::string> RobotontPacket;

class RobotontHW
{
public:
  RobotontHW();
  ~RobotontHW();

  /**
   * \brief Reads the packet from the robot
   * \param packet The packet contents.
   * \return True if the complete packet was placed in \packet packet variable, False if no data has arrived or the
   * packet is not yet complete
   */
  bool readPacket(RobotontPacket& packet);
  void writePacket(const RobotontPacket& packet);

private:
  void connect();
  void checkConnection(const ros::TimerEvent& event);

  serial::Serial serial_;
  std::string packet_buffer_;

  ros::NodeHandle nh_;
  ros::Timer timer_;
  bool reconnect_requested_;
};

typedef std::shared_ptr<RobotontHW> RobotontHWPtr;
}  // namespace robotont

#endif

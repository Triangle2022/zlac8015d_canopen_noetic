#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_srvs/Trigger.h>

#include "zlac8015d_canopen_noetic/socketcan.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

void putU16(can_frame& frame, const int offset, const std::uint16_t value)
{
  frame.data[offset] = static_cast<std::uint8_t>(value & 0xff);
  frame.data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xff);
}

void putI16(can_frame& frame, const int offset, const std::int16_t value)
{
  putU16(frame, offset, static_cast<std::uint16_t>(value));
}

can_frame makeFrame(const canid_t id, const std::uint8_t dlc)
{
  can_frame frame{};
  frame.can_id = id;
  frame.can_dlc = dlc;
  return frame;
}

}  // namespace

class Zlac8015dCanopenNode
{
public:
  Zlac8015dCanopenNode()
    : nh_(), private_nh_("~")
  {
    private_nh_.param<std::string>("can_interface", can_interface_, "can0");
    private_nh_.param<int>("node_id", node_id_, 1);
    private_nh_.param<double>("wheel_radius", wheel_radius_, 0.08);
    private_nh_.param<double>("wheel_separation", wheel_separation_, 0.38);
    private_nh_.param<int>("rpm_limit", rpm_limit_, 1000);
    private_nh_.param<double>("command_timeout", command_timeout_, 0.5);
    private_nh_.param<bool>("publish_encoder", publish_encoder_, true);
    private_nh_.param<double>("encoder_poll_rate", encoder_poll_rate_, 20.0);
    private_nh_.param<double>("encoder_counts_per_rev", encoder_counts_per_rev_, 4096.0);
    private_nh_.param<bool>("auto_enable", auto_enable_, true);
    private_nh_.param<bool>("configure_velocity_mode_on_start", configure_velocity_mode_on_start_, true);
    private_nh_.param<bool>("configure_rpdo_on_start", configure_rpdo_on_start_, true);

    if (node_id_ < 1 || node_id_ > 127)
    {
      throw std::runtime_error("node_id must be in CANopen range 1..127");
    }
    if (wheel_radius_ <= 0.0)
    {
      throw std::runtime_error("wheel_radius must be positive");
    }

    can_.open(can_interface_);

    cmd_sub_ = nh_.subscribe("cmd_vel", 10, &Zlac8015dCanopenNode::cmdVelCallback, this);
    rpm_pub_ = nh_.advertise<std_msgs::Int16MultiArray>("zlac8015d/target_rpm", 10);
    encoder_counts_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("zlac8015d/encoder_counts", 10);
    encoder_angle_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("zlac8015d/encoder_angle", 10);
    enable_srv_ = private_nh_.advertiseService("enable", &Zlac8015dCanopenNode::enableService, this);
    disable_srv_ = private_nh_.advertiseService("disable", &Zlac8015dCanopenNode::disableService, this);
    timer_ = nh_.createTimer(ros::Duration(0.05), &Zlac8015dCanopenNode::timerCallback, this);

    if (configure_velocity_mode_on_start_)
    {
      configureVelocityMode();
    }
    if (configure_rpdo_on_start_)
    {
      configureRpdoVelocity();
    }
    nmtStart();
    if (auto_enable_)
    {
      enableDriver();
    }

    last_command_time_ = ros::Time::now();
    next_encoder_poll_time_ = last_command_time_;
    ROS_INFO_STREAM("ZLAC8015D CANopen node ready on " << can_interface_
                    << " node_id=" << node_id_);
  }

private:
  void nmtStart()
  {
    can_frame frame = makeFrame(0x000, 2);
    frame.data[0] = 0x01;
    frame.data[1] = static_cast<std::uint8_t>(node_id_);
    can_.send(frame);
  }

  void writeSdo8(const std::uint16_t index, const std::uint8_t subindex, const std::uint8_t value)
  {
    can_frame frame = makeFrame(0x600 + node_id_, 8);
    frame.data[0] = 0x2f;
    putU16(frame, 1, index);
    frame.data[3] = subindex;
    frame.data[4] = value;
    can_.send(frame);
    ros::Duration(0.005).sleep();
  }

  void writeSdo16(const std::uint16_t index, const std::uint8_t subindex, const std::uint16_t value)
  {
    can_frame frame = makeFrame(0x600 + node_id_, 8);
    frame.data[0] = 0x2b;
    putU16(frame, 1, index);
    frame.data[3] = subindex;
    putU16(frame, 4, value);
    can_.send(frame);
    ros::Duration(0.005).sleep();
  }

  void writeSdo32(const std::uint16_t index, const std::uint8_t subindex, const std::uint32_t value)
  {
    can_frame frame = makeFrame(0x600 + node_id_, 8);
    frame.data[0] = 0x23;
    putU16(frame, 1, index);
    frame.data[3] = subindex;
    frame.data[4] = static_cast<std::uint8_t>(value & 0xff);
    frame.data[5] = static_cast<std::uint8_t>((value >> 8) & 0xff);
    frame.data[6] = static_cast<std::uint8_t>((value >> 16) & 0xff);
    frame.data[7] = static_cast<std::uint8_t>((value >> 24) & 0xff);
    can_.send(frame);
    ros::Duration(0.005).sleep();
  }

  bool readSdoI32(const std::uint16_t index, const std::uint8_t subindex, std::int32_t& value)
  {
    can_frame request = makeFrame(0x600 + node_id_, 8);
    request.data[0] = 0x40;
    putU16(request, 1, index);
    request.data[3] = subindex;
    can_.send(request);

    const canid_t response_id = 0x580 + node_id_;
    const ros::Time deadline = ros::Time::now() + ros::Duration(0.02);
    while (ros::Time::now() < deadline)
    {
      can_frame response{};
      if (!can_.receive(response, 2))
      {
        continue;
      }
      if (response.can_id != response_id || response.can_dlc < 8)
      {
        continue;
      }
      if (response.data[1] != static_cast<std::uint8_t>(index & 0xff) ||
          response.data[2] != static_cast<std::uint8_t>((index >> 8) & 0xff) ||
          response.data[3] != subindex)
      {
        continue;
      }
      if ((response.data[0] & 0xf0) != 0x40)
      {
        continue;
      }

      const std::uint32_t raw =
          static_cast<std::uint32_t>(response.data[4]) |
          (static_cast<std::uint32_t>(response.data[5]) << 8) |
          (static_cast<std::uint32_t>(response.data[6]) << 16) |
          (static_cast<std::uint32_t>(response.data[7]) << 24);
      value = static_cast<std::int32_t>(raw);
      return true;
    }

    return false;
  }

  void configureVelocityMode()
  {
    writeSdo8(0x6060, 0x00, 0x03);
    writeSdo32(0x6083, 0x01, 100);
    writeSdo32(0x6083, 0x02, 100);
    writeSdo32(0x6084, 0x01, 100);
    writeSdo32(0x6084, 0x02, 100);
    writeSdo16(0x200f, 0x00, 1);
  }

  void configureRpdoVelocity()
  {
    writeSdo8(0x1601, 0x00, 0);
    writeSdo8(0x1401, 0x02, 0xfe);
    writeSdo32(0x1601, 0x01, 0x60ff0320);
    writeSdo8(0x1601, 0x00, 1);
  }

  void enableDriver()
  {
    writeControlword(0x0006);
    writeControlword(0x0007);
    writeControlword(0x000f);
    enabled_ = true;
  }

  void disableDriver()
  {
    setTargetRpm(0, 0);
    writeControlword(0x0006);
    enabled_ = false;
  }

  void writeControlword(const std::uint16_t value)
  {
    writeSdo16(0x6040, 0x00, value);
  }

  bool enableService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    try
    {
      nmtStart();
      enableDriver();
      response.success = true;
      response.message = "driver enabled";
    }
    catch (const std::exception& error)
    {
      response.success = false;
      response.message = error.what();
    }
    return true;
  }

  bool disableService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    try
    {
      disableDriver();
      response.success = true;
      response.message = "driver disabled";
    }
    catch (const std::exception& error)
    {
      response.success = false;
      response.message = error.what();
    }
    return true;
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    const double left_mps = msg->linear.x - msg->angular.z * wheel_separation_ * 0.5;
    const double right_mps = msg->linear.x + msg->angular.z * wheel_separation_ * 0.5;

    const std::int16_t left_rpm = mpsToRpm(left_mps);
    const std::int16_t right_rpm = mpsToRpm(right_mps);

    setTargetRpm(left_rpm, right_rpm);
    last_command_time_ = ros::Time::now();
  }

  std::int16_t mpsToRpm(const double mps) const
  {
    const double rpm = (mps / (2.0 * kPi * wheel_radius_)) * 60.0;
    const int rounded = static_cast<int>(std::lround(rpm));
    return static_cast<std::int16_t>(std::max(-rpm_limit_, std::min(rpm_limit_, rounded)));
  }

  void setTargetRpm(const std::int16_t left_rpm, const std::int16_t right_rpm)
  {
    target_left_rpm_ = left_rpm;
    target_right_rpm_ = right_rpm;
    sendTargetRpm(left_rpm, right_rpm);
    publishTarget(left_rpm, right_rpm);
  }

  void sendTargetRpm(const std::int16_t left_rpm, const std::int16_t right_rpm)
  {
    can_frame frame = makeFrame(0x300 + node_id_, 4);
    putI16(frame, 0, left_rpm);
    putI16(frame, 2, right_rpm);
    can_.send(frame);
  }

  void publishTarget(const std::int16_t left_rpm, const std::int16_t right_rpm)
  {
    std_msgs::Int16MultiArray msg;
    msg.data.push_back(left_rpm);
    msg.data.push_back(right_rpm);
    rpm_pub_.publish(msg);
  }

  void timerCallback(const ros::TimerEvent& event)
  {
    const ros::Time now = event.current_real.isZero() ? ros::Time::now() : event.current_real;

    if (enabled_ && command_timeout_ > 0.0 &&
        (now - last_command_time_).toSec() > command_timeout_)
    {
      setTargetRpm(0, 0);
      last_command_time_ = now;
    }

    pollAndPublishEncoder(now);
  }

  void pollAndPublishEncoder(const ros::Time& now)
  {
    if (!publish_encoder_ || encoder_poll_rate_ <= 0.0 || now < next_encoder_poll_time_)
    {
      return;
    }

    next_encoder_poll_time_ = now + ros::Duration(1.0 / encoder_poll_rate_);

    std::int32_t left_counts = 0;
    std::int32_t right_counts = 0;
    const bool left_ok = readSdoI32(0x6064, 0x01, left_counts);
    const bool right_ok = readSdoI32(0x6064, 0x02, right_counts);
    if (!left_ok || !right_ok)
    {
      ROS_WARN_THROTTLE(1.0, "failed to read ZLAC8015D encoder position via SDO 0x6064");
      return;
    }

    std_msgs::Int32MultiArray counts_msg;
    counts_msg.data.push_back(left_counts);
    counts_msg.data.push_back(right_counts);
    encoder_counts_pub_.publish(counts_msg);

    std_msgs::Float64MultiArray angle_msg;
    angle_msg.data.push_back(countsToRadians(left_counts));
    angle_msg.data.push_back(countsToRadians(right_counts));
    encoder_angle_pub_.publish(angle_msg);
  }

  double countsToRadians(const std::int32_t counts) const
  {
    return (static_cast<double>(counts) / encoder_counts_per_rev_) * 2.0 * kPi;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher rpm_pub_;
  ros::Publisher encoder_counts_pub_;
  ros::Publisher encoder_angle_pub_;
  ros::ServiceServer enable_srv_;
  ros::ServiceServer disable_srv_;
  ros::Timer timer_;
  zlac8015d::SocketCan can_;

  std::string can_interface_;
  int node_id_{1};
  double wheel_radius_{0.08};
  double wheel_separation_{0.38};
  int rpm_limit_{1000};
  double command_timeout_{0.5};
  bool publish_encoder_{true};
  double encoder_poll_rate_{20.0};
  double encoder_counts_per_rev_{4096.0};
  bool auto_enable_{true};
  bool configure_velocity_mode_on_start_{true};
  bool configure_rpdo_on_start_{true};
  bool enabled_{false};
  ros::Time last_command_time_;
  ros::Time next_encoder_poll_time_;
  std::int16_t target_left_rpm_{0};
  std::int16_t target_right_rpm_{0};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zlac8015d_canopen_node");

  try
  {
    Zlac8015dCanopenNode node;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("zlac8015d_canopen_node failed: " << error.what());
    return 1;
  }

  return 0;
}

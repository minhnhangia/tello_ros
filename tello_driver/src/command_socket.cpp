#include "tello_driver_node.hpp"

namespace tello_driver
{

  CommandSocket::CommandSocket(TelloDriverNode *driver, std::string drone_ip,
                               unsigned short drone_port, unsigned short command_port) :
    TelloSocket(driver, command_port),
    remote_endpoint_(asio::ip::address_v4::from_string(drone_ip), drone_port),
    send_time_(rclcpp::Time(0L, RCL_ROS_TIME))
  {
    buffer_ = std::vector<unsigned char>(1024);
    listen();
  }

  void CommandSocket::timeout()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    receiving_ = false;

    if (waiting_) {
      complete_command(tello_msgs::msg::TelloResponse::TIMEOUT, "error: command timed out");
    }
  }

  bool CommandSocket::waiting()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return waiting_;
  }

  rclcpp::Time CommandSocket::send_time()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return send_time_;
  }

  void CommandSocket::initiate_command(std::string command, bool respond)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (waiting_) return;
    
    RCLCPP_DEBUG(driver_->get_logger(), "Sending '%s'...", command.c_str());

    try
    {
      socket_.send_to(asio::buffer(command), remote_endpoint_);
      send_time_ = driver_->now();

      // Wait for a response for all commands except "rc"
      if (command.rfind("rc", 0) != 0) 
      {
        respond_ = respond;
        waiting_ = true;
      }
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(driver_->get_logger(), "Failed to send command '%s': %s", command.c_str(), e.what());
      if (respond) 
      {
        complete_command(tello_msgs::msg::TelloResponse::ERROR, std::string("error: ") + e.what());
      }
    }
  }

  void CommandSocket::query_ext_tof()
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (waiting_) return;
    
    RCLCPP_DEBUG(driver_->get_logger(), "Querying EXT TOF sensor...");

    try
    {
      socket_.send_to(asio::buffer("EXT tof?"), remote_endpoint_);
      send_time_ = driver_->now();
      respond_ = false;  // Handle response directly, don't publish to tello_response
      waiting_ = true;
      waiting_ext_tof_ = true;
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(driver_->get_logger(), "Failed to query EXT TOF: %s", e.what());
    }
  }

  void CommandSocket::handle_ext_tof_response(const std::string& response)
  {
    // Parse EXT TOF response format: "tof XXX" where XXX is distance in millimeters
    if (response.size() >= 5 && response.substr(0, 4) == "tof ") {
      try {
        std::string distance_str = response.substr(4);  // Skip "tof " prefix
        int distance_mm = std::stoi(distance_str);
        
        // Convert to meters for ROS standard units
        float distance_m = distance_mm / 1000.0f;
        
        // Create and populate Range message
        sensor_msgs::msg::Range range_msg;
        range_msg.header.stamp = driver_->now();
        range_msg.header.frame_id = "ext_tof_link";  // Forward-facing external sensor frame
        range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_msg.field_of_view = 0.05f;  // ~3 degrees typical for TOF sensors
        range_msg.min_range = 0.03f;      // 30mm minimum range
        range_msg.max_range = 8.0f;       // 8m maximum range (typical for Tello expansion kit)
        range_msg.range = distance_m;
        
        // Publish the range measurement
        driver_->ext_tof_pub_->publish(range_msg);
        
        RCLCPP_DEBUG(driver_->get_logger(), "EXT TOF: %dmm (%.3fm)", distance_mm, distance_m);
        
        // Complete the command with success
        complete_command(tello_msgs::msg::TelloResponse::OK, response);
        
      } catch (const std::exception& e) {
        RCLCPP_ERROR(driver_->get_logger(), "Failed to parse EXT TOF response '%s': %s", 
                     response.c_str(), e.what());
        complete_command(tello_msgs::msg::TelloResponse::ERROR, "error: invalid EXT TOF response");
      }
    } else {
      RCLCPP_ERROR(driver_->get_logger(), "Invalid EXT TOF response format: '%s'", response.c_str());
      complete_command(tello_msgs::msg::TelloResponse::ERROR, "error: invalid EXT TOF response format");
    }
    
    waiting_ = false;
  }

  void CommandSocket::complete_command(uint8_t rc, std::string str)
  {
    if (respond_) {
      tello_msgs::msg::TelloResponse response_msg;
      response_msg.rc = rc;
      response_msg.str = str;
      driver_->tello_response_pub_->publish(response_msg);
    }
    waiting_ = false;
  }

  void CommandSocket::process_packet(size_t r)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    receive_time_ = driver_->now();

    if (!receiving_) {
      receiving_ = true;
    }

    std::string str = std::string(buffer_.begin(), buffer_.begin() + r);
    if (waiting_) {
      RCLCPP_DEBUG(driver_->get_logger(), "Received '%s'", str.c_str());
      
      // Handle EXT TOF responses specially
      if (waiting_ext_tof_) {
        handle_ext_tof_response(str);
        waiting_ext_tof_ = false;
      } else {
        complete_command(str == "error" ? tello_msgs::msg::TelloResponse::ERROR : tello_msgs::msg::TelloResponse::OK,
                         str);
      }
    } else {
      RCLCPP_WARN(driver_->get_logger(), "Unexpected '%s'", str.c_str());
    }
  }

} // namespace tello_driver

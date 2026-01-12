#include "tello_driver_node.hpp"

#include <algorithm>
#include <cctype>

namespace tello_driver
{

  CommandSocket::CommandSocket(TelloDriverNode *driver, std::string drone_ip,
                               unsigned short drone_port, unsigned short command_port) :
    TelloSocket(driver, command_port),
    remote_endpoint_(asio::ip::address_v4::from_string(drone_ip), drone_port),
    send_time_(rclcpp::Time(0L, RCL_ROS_TIME)),
    ext_tof_send_time_(rclcpp::Time(0L, RCL_ROS_TIME))
  {
    buffer_ = std::vector<unsigned char>(1024);
    listen();
  }

  void CommandSocket::timeout()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    receiving_ = false;

    if (waiting_) {
      last_result_ = CommandResult::TIMEOUT;
      complete_command(tello_msgs::msg::TelloResponse::TIMEOUT, "error: command timed out");
    }
    
    // Reset EXT TOF waiting state on timeout
    if (waiting_ext_tof_) {
      waiting_ext_tof_ = false;
    }
  }

  bool CommandSocket::waiting()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return waiting_;
  }

  bool CommandSocket::is_busy()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // Drone can only handle one command at a time - check both regular and EXT TOF
    return waiting_ || waiting_ext_tof_;
  }

  rclcpp::Time CommandSocket::send_time()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return send_time_;
  }

  bool CommandSocket::waiting_ext_tof()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return waiting_ext_tof_;
  }

  void CommandSocket::timeout_ext_tof()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // Only reset EXT TOF state - don't touch receiving_ or waiting_
    // This prevents corrupting the main command state when a non-critical
    // EXT TOF query times out
    if (waiting_ext_tof_) {
      RCLCPP_DEBUG(driver_->get_logger(), "EXT TOF query timed out");
      waiting_ext_tof_ = false;
    }
  }

  rclcpp::Time CommandSocket::ext_tof_send_time()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return ext_tof_send_time_;
  }

  CommandResult CommandSocket::get_last_result()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return last_result_;
  }

  void CommandSocket::clear_last_result()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_result_ = CommandResult::NONE;
  }

  void CommandSocket::initiate_command(std::string command, bool respond)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    // Cannot send if ANY command is in progress (regular or EXT TOF)
    if (waiting_ || waiting_ext_tof_) 
    {
        RCLCPP_DEBUG(driver_->get_logger(), "Busy, cannot send '%s'", command.c_str());
        return;
    };
    
    // Clear previous result before sending new command
    last_result_ = CommandResult::NONE;
    
    RCLCPP_DEBUG(driver_->get_logger(), "Sending '%s'...", command.c_str());

    try
    {
      socket_.send_to(asio::buffer(command), remote_endpoint_);
      send_time_ = driver_->now();
      last_command_ = command;

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

    // Don't interfere with other commands - EXT TOF is non-critical
    if (waiting_) {
      RCLCPP_DEBUG(driver_->get_logger(), "Skipping EXT TOF query - command in progress");
      return;
    }
    
    RCLCPP_DEBUG(driver_->get_logger(), "Querying EXT TOF sensor...");

    try
    {
      socket_.send_to(asio::buffer("EXT tof?"), remote_endpoint_);
      ext_tof_send_time_ = driver_->now();  // Use separate timing for EXT TOF
      respond_ = false;  // Handle response directly, don't publish to tello_response
      // DO NOT set waiting_ = true - let other commands proceed normally
      waiting_ext_tof_ = true;  // Only track EXT TOF state separately
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(driver_->get_logger(), "Failed to query EXT TOF: %s", e.what());
      waiting_ext_tof_ = false;  // Reset on error
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
        
      } catch (const std::exception& e) {
        RCLCPP_ERROR(driver_->get_logger(), "Failed to parse EXT TOF response '%s': %s", 
                     response.c_str(), e.what());
      }
    } else {
      RCLCPP_ERROR(driver_->get_logger(), "Invalid EXT TOF response format: '%s'", response.c_str());
    }
    
    // Note: Do NOT call complete_command() or set waiting_ = false since
    // EXT TOF queries don't block other commands
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
    
    // Handle TOF responses (regardless of waiting_ext_tof_ state to avoid "Unexpected" warnings)
    if (str.size() >= 5 && str.substr(0, 4) == "tof ") 
    {
      RCLCPP_DEBUG(driver_->get_logger(), "Received EXT TOF response: '%s'", str.c_str());
      handle_ext_tof_response(str);
      if (waiting_ext_tof_) 
      {
        waiting_ext_tof_ = false;
      }
    }
    else if (waiting_)
    {
      RCLCPP_DEBUG(driver_->get_logger(), "Received '%s'", str.c_str());
      const bool ok = (str != "error");
      
      // Track result for response-gated state machines
      last_result_ = ok ? CommandResult::OK : CommandResult::ERROR;
      
      complete_command(ok ? tello_msgs::msg::TelloResponse::OK : tello_msgs::msg::TelloResponse::ERROR,
                       str);

      // Handle downvision camera switch when command succeeds
      if (ok) 
      {
        // Normalize command to lowercase for matching
        std::string cmd = last_command_;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), [](unsigned char c){ return std::tolower(c); });
        if (cmd.rfind("downvision", 0) == 0) 
        {
          // Expect format: "downvision 0" or "downvision 1"
          // Extract the argument after space
          bool active = false;
          auto pos = cmd.find_first_of(' ');
          if (pos != std::string::npos) {
            std::string arg = cmd.substr(pos + 1);
            active = (arg == "1" || arg == "on" || arg == "true");
          }
          driver_->set_downvision_active(active);
        }
      }
    } 
    else 
    {
      RCLCPP_WARN(driver_->get_logger(), "Unexpected '%s'", str.c_str());
    }
  }

} // namespace tello_driver

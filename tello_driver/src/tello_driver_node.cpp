#include "tello_driver_node.hpp"

#include "ros2_shared/context_macros.hpp"
#include "camera_calibration_parsers/parse.hpp"
#include <opencv2/imgproc.hpp>

using asio::ip::udp;

namespace tello_driver
{

#define TELLO_DRIVER_ALL_PARAMS \
  CXT_MACRO_MEMBER(drone_ip, std::string, std::string("192.168.10.1")) /* Send commands to this IP address */ \
  CXT_MACRO_MEMBER(drone_port, int, 8889)                 /* Send commands to this port */ \
  CXT_MACRO_MEMBER(command_port, int, 38065)              /* Send commands from this port */ \
  CXT_MACRO_MEMBER(data_port, int, 8890)                  /* Flight data will arrive at this port */ \
  CXT_MACRO_MEMBER(video_port, int, 11111)                /* Video data will arrive at this port */ \
  CXT_MACRO_MEMBER(video_stream_url, std::string, std::string("udp://0.0.0.0:11111")) /* OpenCV/FFmpeg URL */ \
  CXT_MACRO_MEMBER(camera_info_path, std::string, \
    "install/tello_driver/share/tello_driver/cfg/camera_info.yaml") /* Forward camera calibration path */ \
  CXT_MACRO_MEMBER(camera_info_path_down, std::string, \
    "install/tello_driver/share/tello_driver/cfg/camera_down_info.yaml") /* Downward camera calibration path (optional) */ \
  CXT_MACRO_MEMBER(odom_frame_id, std::string, std::string("odom")) /* Odometry frame ID */ \
  CXT_MACRO_MEMBER(base_frame_id, std::string, std::string("base_link")) /* Base frame ID */ \
  CXT_MACRO_MEMBER(camera_frame_id_forward, std::string, std::string("camera_frame")) /* Forward camera frame ID */ \
  CXT_MACRO_MEMBER(camera_frame_id_down, std::string, std::string("camera_down_frame")) /* Downward camera frame ID */ \
  CXT_MACRO_MEMBER(publish_down_as_mono, bool, false) \
  /* End of list */

  struct TelloDriverContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    CXT_MACRO_DEFINE_MEMBERS(TELLO_DRIVER_ALL_PARAMS)
  };

  constexpr int32_t STATE_TIMEOUT = 4;      // We stopped receiving telemetry
  constexpr int32_t VIDEO_TIMEOUT = 4;      // We stopped receiving video
  constexpr int32_t KEEP_ALIVE = 12;        // We stopped receiving input from other ROS nodes
  constexpr int32_t COMMAND_TIMEOUT = 9;    // Drone didn't respond to a command

  TelloDriverNode::TelloDriverNode(const rclcpp::NodeOptions &options) :
    Node("tello_driver", options)
  {
    // ROS publishers
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos_profile);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos_profile);
    flight_data_pub_ = create_publisher<tello_msgs::msg::FlightData>("flight_data", qos_profile);
    tello_response_pub_ = create_publisher<tello_msgs::msg::TelloResponse>("tello_response", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", qos_profile);
    ext_tof_pub_ = create_publisher<sensor_msgs::msg::Range>("ext_tof", qos_profile);

    // ROS service
    command_srv_ = create_service<tello_msgs::srv::TelloAction>(
      "tello_action", std::bind(&TelloDriverNode::command_callback, this,
                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // ROS subscription
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&TelloDriverNode::cmd_vel_callback, this, std::placeholders::_1));

    // ROS timers
    using namespace std::chrono_literals;
    spin_timer_ = create_wall_timer(1s, std::bind(&TelloDriverNode::timer_callback, this));
    ext_tof_timer_ = create_wall_timer(200ms, std::bind(&TelloDriverNode::ext_tof_timer_callback, this));  // 5Hz

    // Parameters - Allocate the parameter context as a local variable because it is not used outside this routine
    TelloDriverContext cxt{};
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(TELLO_DRIVER_ALL_PARAMS, [this]()
    {})

    // NOTE: This is not setup to dynamically update parameters after ths node is running.

    RCLCPP_INFO(get_logger(), "Drone at %s:%d", cxt.drone_ip_.c_str(), cxt.drone_port_);
    RCLCPP_INFO(get_logger(), "Listening for command responses on localhost:%d", cxt.command_port_);
    RCLCPP_INFO(get_logger(), "Listening for data on localhost:%d", cxt.data_port_);
    RCLCPP_INFO(get_logger(), "Listening for video on localhost:%d", cxt.video_port_);

    // Store frame IDs for odometry
    odom_frame_id_ = cxt.odom_frame_id_;
    base_frame_id_ = cxt.base_frame_id_;

    // Sockets
    command_socket_ = std::make_unique<CommandSocket>(this, cxt.drone_ip_, cxt.drone_port_, cxt.command_port_);
    state_socket_ = std::make_unique<StateSocket>(this, cxt.data_port_);
    // OpenCV VideoCapture pipeline replaces VideoSocket
    publish_down_as_mono_ = cxt.publish_down_as_mono_;
    camera_info_path_forward_ = cxt.camera_info_path_;
    camera_info_path_down_ = cxt.camera_info_path_down_;
    frame_id_forward_ = cxt.camera_frame_id_forward_;
    frame_id_down_ = cxt.camera_frame_id_down_;
    video_stream_url_ = cxt.video_stream_url_;

    // Load forward camera info initially
    load_camera_info();

    // Create a timer to grab frames; 33ms ~ 30 FPS
    using namespace std::chrono_literals;
    video_timer_ = create_wall_timer(33ms, std::bind(&TelloDriverNode::video_timer_callback, this));
  }

  TelloDriverNode::~TelloDriverNode()
  {
  }

  void TelloDriverNode::command_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<tello_msgs::srv::TelloAction::Request> request,
    std::shared_ptr<tello_msgs::srv::TelloAction::Response> response)
  {
    (void) request_header;
    if (!state_socket_->receiving() || !video_receiving_) {
      RCLCPP_WARN(get_logger(), "Not connected, dropping '%s'", request->cmd.c_str());
      response->rc = response->ERROR_NOT_CONNECTED;
    } else if (command_socket_->waiting()) {
      RCLCPP_WARN(get_logger(), "Busy, dropping '%s'", request->cmd.c_str());
      response->rc = response->ERROR_BUSY;
    } else {
      RCLCPP_INFO(get_logger(), "Command received: '%s'", request->cmd.c_str());
      command_socket_->initiate_command(request->cmd, true);
      response->rc = response->OK;
    }
  }

  void TelloDriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // TODO cmd_vel should specify velocity, not joystick position
    if (!command_socket_->waiting()) {
      std::ostringstream rc;
      rc << "rc " << static_cast<int>(round(msg->linear.y * 100))
         << " " << static_cast<int>(round(msg->linear.x * 100))
         << " " << static_cast<int>(round(msg->linear.z * 100))
         << " " << static_cast<int>(round(msg->angular.z * 100));
      command_socket_->initiate_command(rc.str(), false);
    }
  }
  void TelloDriverNode::set_downvision_active(bool active)
  {
    // Toggle active camera; reload calibration
    downvision_active_ = active;
    load_camera_info();
  }

  // Do work every 1s
  void TelloDriverNode::timer_callback()
  {
    //====
    // Startup
    //====

    if (!state_socket_->receiving() && !command_socket_->waiting())
    {
      // First command to the drone must be "command"
      command_socket_->initiate_command("command", false);
      return;
    }

    // Configure/start video using Tello commands and OpenCV VideoCapture
    if (state_socket_->receiving() && !video_opened_ && !command_socket_->waiting()) {
      // Configure video settings before starting stream on the drone
      static bool fps_configured = false;
      if (!fps_configured) {
        command_socket_->initiate_command("setfps middle", false);
        fps_configured = true;
        RCLCPP_INFO(get_logger(), "Set video FPS to middle");
        return;
      }
      // Start video on the drone
      command_socket_->initiate_command("streamon", false);
      // Try to open local receiver after sending streamon
      if (!video_open_attempted_) {
        video_open_attempted_ = true;
        // Prefer FFmpeg backend to ensure H264 over UDP support
        if (video_capture_.open(video_stream_url_, cv::CAP_FFMPEG)) {
          video_opened_ = video_capture_.isOpened();
          if (video_opened_) {
            RCLCPP_INFO(get_logger(), "Opened video stream via OpenCV: %s", video_stream_url_.c_str());
          } else {
            RCLCPP_ERROR(get_logger(), "Failed to open video stream: %s", video_stream_url_.c_str());
          }
        } else {
          RCLCPP_ERROR(get_logger(), "OpenCV failed to open stream URL: %s", video_stream_url_.c_str());
        }
      }
      return;  
    }

    //====
    // Timeouts
    //====

    bool timeout = false;

    if (command_socket_->waiting() && now() - command_socket_->send_time() > rclcpp::Duration(COMMAND_TIMEOUT, 0)) {
      RCLCPP_ERROR(get_logger(), "Command timed out");
      command_socket_->timeout();
      timeout = true;
    }

    if (state_socket_->receiving() && now() - state_socket_->receive_time() > rclcpp::Duration(STATE_TIMEOUT, 0)) {
      RCLCPP_ERROR(get_logger(), "No state received for 5s");
      state_socket_->timeout();
      timeout = true;
    }

    if (video_opened_ && video_receiving_ && now() - video_receive_time_ > rclcpp::Duration(VIDEO_TIMEOUT, 0)) {
      RCLCPP_ERROR(get_logger(), "No video received for 5s");
      video_receiving_ = false;
      timeout = true;
    }

    if (timeout) {
      return;
    }

    //====
    // Keep-alive, drone will auto-land if it hears nothing for 15s
    //====

    if (state_socket_->receiving() && !command_socket_->waiting() &&
        now() - command_socket_->send_time() > rclcpp::Duration(KEEP_ALIVE, 0)) {
      command_socket_->initiate_command("rc 0 0 0 0", false);
      return;
    }

    if (!is_first_init_ && state_socket_->receiving() && video_receiving_)
    {
      RCLCPP_INFO(get_logger(), "Tello driver initialized!");
      is_first_init_ = true;
    }
  }

  void TelloDriverNode::ext_tof_timer_callback()
  {
    if (!is_first_init_)
    {
      return;
    }

    // Check for EXT TOF timeout (1 second timeout for non-critical queries)
    if (command_socket_->waiting_ext_tof() && 
        now() - command_socket_->ext_tof_send_time() > rclcpp::Duration(1, 0))
    {
      // RCLCPP_WARN(get_logger(), "EXT TOF query timed out");
      // Reset the EXT TOF waiting state
      command_socket_->timeout();
      return;
    }

    // EXT TOF Sensor Queries (forward-facing external sensor) at 5Hz
    if (state_socket_->receiving() && video_receiving_ && 
        !command_socket_->waiting() && !command_socket_->waiting_ext_tof() &&
        ext_tof_pub_->get_subscription_count() > 0) 
    {
      command_socket_->query_ext_tof();
    }
  }

  void TelloDriverNode::publish_odometry(const tello_msgs::msg::FlightData& flight_data)
  {
    // Only publish if there are subscribers
    if (count_subscribers(odom_pub_->get_topic_name()) == 0) 
    {
      return;
    }

    nav_msgs::msg::Odometry odom_msg;
    
    // Header
    odom_msg.header.stamp = flight_data.header.stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    // Position data not available (set to 0)
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (convert from degrees to quaternion)
    tf2::Quaternion q;
    q.setRPY(
      flight_data.roll * M_PI / 180.0,   // roll: degrees to radians
      flight_data.pitch * M_PI / 180.0,  // pitch: degrees to radians
      flight_data.yaw * M_PI / 180.0     // yaw: degrees to radians
    );
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // Velocity (convert from cm/s to m/s)
    odom_msg.twist.twist.linear.x = flight_data.vgx / 100.0;   // cm/s to m/s
    odom_msg.twist.twist.linear.y = flight_data.vgy / 100.0;   // cm/s to m/s
    odom_msg.twist.twist.linear.z = flight_data.vgz / 100.0;   // cm/s to m/s
    
    // Angular velocity (we don't have this data from Tello, set to 0)
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Covariance matrices (set to unknown/high uncertainty)
    // Position covariance (6x6 matrix, but stored as 36-element array)
    std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
    odom_msg.pose.covariance[0] = 0.1;   // x variance
    odom_msg.pose.covariance[7] = 0.1;   // y variance  
    odom_msg.pose.covariance[14] = 0.1;  // z variance
    odom_msg.pose.covariance[21] = 0.1;  // roll variance
    odom_msg.pose.covariance[28] = 0.1;  // pitch variance
    odom_msg.pose.covariance[35] = 0.1;  // yaw variance

    // Velocity covariance (6x6 matrix)
    std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
    odom_msg.twist.covariance[0] = 0.1;   // vx variance
    odom_msg.twist.covariance[7] = 0.1;   // vy variance
    odom_msg.twist.covariance[14] = 0.1;  // vz variance
    odom_msg.twist.covariance[21] = 0.1;  // angular x variance
    odom_msg.twist.covariance[28] = 0.1;  // angular y variance
    odom_msg.twist.covariance[35] = 0.1;  // angular z variance

    // Publish the odometry message
    odom_pub_->publish(odom_msg);
  }

  //=============================
  // OpenCV video timer callback
  //=============================
  void TelloDriverNode::video_timer_callback()
  {
    if (!video_opened_) {
      return;
    }

    // Grab frame
    if (!video_capture_.read(video_frame_)) {
      // read() returns false on failure/end; keep trying
      return;
    }

    if (video_frame_.empty()) {
      return;
    }

    // Mark receiving and update timestamp
    video_receiving_ = true;
    video_receive_time_ = now();

    // Publish image if there are subscribers
    if (count_subscribers(image_pub_->get_topic_name()) > 0) {
      // OpenCV provides BGR; use bgr8 encoding
      std_msgs::msg::Header header{};
      header.stamp = video_receive_time_;
      header.frame_id = downvision_active_ ? frame_id_down_ : frame_id_forward_;

      // Optionally publish mono for downvision by converting BGR->GRAY
      const bool publish_mono = (downvision_active_ && publish_down_as_mono_);
      if (publish_mono) {
        cv::Mat gray;
        cv::cvtColor(video_frame_, gray, cv::COLOR_BGR2GRAY);
        cv_bridge::CvImage cv_image{header, sensor_msgs::image_encodings::MONO8, gray};
        sensor_msgs::msg::Image msg;
        cv_image.toImageMsg(msg);
        image_pub_->publish(msg);
      } else {
        cv_bridge::CvImage cv_image{header, sensor_msgs::image_encodings::BGR8, video_frame_};
        sensor_msgs::msg::Image msg;
        cv_image.toImageMsg(msg);
        image_pub_->publish(msg);
      }
    }

    // Publish CameraInfo if there are subscribers
    if (count_subscribers(camera_info_pub_->get_topic_name()) > 0) {
      camera_info_msg_.width = video_frame_.cols;
      camera_info_msg_.height = video_frame_.rows;
      camera_info_msg_.header.stamp = video_receive_time_;
      camera_info_msg_.header.frame_id = downvision_active_ ? frame_id_down_ : frame_id_forward_;
      camera_info_pub_->publish(camera_info_msg_);
    }
  }

  void TelloDriverNode::load_camera_info()
  {
    const std::string &path = (downvision_active_ && !camera_info_path_down_.empty())
      ? camera_info_path_down_ : camera_info_path_forward_;

    std::string camera_name;
    sensor_msgs::msg::CameraInfo loaded;
    if (camera_calibration_parsers::readCalibration(path, camera_name, loaded))
    {
      camera_info_msg_ = loaded;
      RCLCPP_INFO(get_logger(), "Parsed camera info for '%s' from '%s'", camera_name.c_str(), path.c_str());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Cannot read camera info from '%s' (downvision=%s). Using existing calibration.",
                  path.c_str(), downvision_active_ ? "true" : "false");
    }
    camera_info_msg_.header.frame_id = downvision_active_ ? frame_id_down_ : frame_id_forward_;
  }

} // namespace tello_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tello_driver::TelloDriverNode)
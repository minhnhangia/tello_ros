#include "tello_driver_node.hpp"

extern "C" {
#include <libavutil/frame.h>
#include <libavutil/log.h>
#include <libavutil/pixfmt.h>
}
#include <opencv2/highgui.hpp>

#include "camera_calibration_parsers/parse.hpp"

namespace tello_driver
{

  // Notes on Tello video:
  // -- frames are always 960x720.
  // -- frames are split into UDP packets of length 1460.
  // -- normal frames are ~10k, or about 8 UDP packets.
  // -- keyframes are ~35k, or about 25 UDP packets.
  // -- keyframes are always preceded by an 8-byte UDP packet and a 13-byte UDP packet -- markers?
  // -- the h264 parser will consume the 8-byte packet, the 13-byte packet and the entire keyframe without
  //    generating a frame. Presumably the keyframe is stored in the parser and referenced later.

  VideoSocket::VideoSocket(TelloDriverNode *driver,
                           unsigned short video_port,
                           const std::string &camera_info_path_forward,
                           const std::string &camera_info_path_down,
                           const std::string &frame_id_forward,
                           const std::string &frame_id_down,
                           bool publish_down_as_mono) :
    TelloSocket(driver, video_port),
    camera_info_path_forward_(camera_info_path_forward),
    camera_info_path_down_(camera_info_path_down),
    frame_id_forward_(frame_id_forward),
    frame_id_down_(frame_id_down),
    publish_down_as_mono_(publish_down_as_mono)
  {
    // Suppress FFmpeg log messages
    av_log_set_level(AV_LOG_QUIET);

    buffer_ = std::vector<unsigned char>(2048);
    seq_buffer_ = std::vector<unsigned char>(65536);

    // Load initial (forward) camera info
    set_downvision_active(false);

    listen();
  }

  void VideoSocket::set_downvision_active(bool active)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    downvision_active_ = active;

    const std::string &path = downvision_active_ && !camera_info_path_down_.empty()
                                ? camera_info_path_down_
                                : camera_info_path_forward_;

    std::string camera_name;
    sensor_msgs::msg::CameraInfo loaded;
    if (camera_calibration_parsers::readCalibration(path, camera_name, loaded))
    {
      camera_info_msg_ = loaded;
      RCLCPP_INFO(driver_->get_logger(), "Parsed camera info for '%s' from '%s'", camera_name.c_str(), path.c_str());
    }
    else
    {
      // Fallback: preserve previous but adjust width/height if we know typical sizes
      RCLCPP_WARN(driver_->get_logger(), "Cannot read camera info from '%s' (downvision=%s). Using existing calibration.",
                  path.c_str(), downvision_active_ ? "true" : "false");
    }

    // Ensure frame_id is set appropriately; it will be overwritten per-publish as well
    camera_info_msg_.header.frame_id = downvision_active_ ? frame_id_down_ : frame_id_forward_;
  }

  // Process a video packet from the drone
  void VideoSocket::process_packet(size_t r)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    receive_time_ = driver_->now();

    if (!receiving_) {
      // First packet
      RCLCPP_INFO(driver_->get_logger(), "Receiving video");
      receiving_ = true;
      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
    }

    if (seq_buffer_next_ + r >= seq_buffer_.size()) {
      RCLCPP_ERROR(driver_->get_logger(), "Video buffer overflow, dropping sequence");
      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
      return;
    }

    std::copy(buffer_.begin(), buffer_.begin() + r, seq_buffer_.begin() + seq_buffer_next_);
    seq_buffer_next_ += r;
    seq_buffer_num_packets_++;

    // If the packet is < 1460 bytes then it's the last packet in the sequence
    if (r < 1460) {
      decode_frames();

      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
    }
  }

  // Decode frames
  void VideoSocket::decode_frames()
  {
    size_t next = 0;

    try
    {
      while (next < seq_buffer_next_)
      {
        // Parse h264
        auto result = decoder_.parse(seq_buffer_.data() + next, seq_buffer_next_ - next);

        // Is a frame available?
        if (result.frame)
        {
          // Decode the frame
          const AVFrame &frame = *result.frame;

          // Build cv::Mat for publishing. If downvision is active and configured to publish MONO8,
          // use the luma plane directly; otherwise convert to RGB.
          cv::Mat mat;
          std::string encoding;
          if (downvision_active_ && publish_down_as_mono_ && frame.format == AV_PIX_FMT_YUV420P)
          {
            // Use Y plane (grayscale). Account for possible stride.
            if (frame.width <= 0 || frame.height <= 0 || frame.data[0] == nullptr || frame.linesize[0] <= 0) {
              RCLCPP_WARN(driver_->get_logger(), "Skipping invalid MONO frame w=%d h=%d ls=%d", frame.width, frame.height, frame.linesize[0]);
              encoding.clear();
            } else {
              mat = cv::Mat{frame.height, frame.width, CV_8UC1, frame.data[0], static_cast<size_t>(frame.linesize[0])};
              encoding = sensor_msgs::image_encodings::MONO8;
            }
          }
          else
          {
            // Convert pixels from YUV420P (or current pix_fmt) to RGB24
            if (frame.width <= 0 || frame.height <= 0) {
              RCLCPP_WARN(driver_->get_logger(), "Skipping invalid RGB frame w=%d h=%d", frame.width, frame.height);
              encoding.clear();
            } else {
              int required = converter_.predict_size(frame.width, frame.height);
              if (required <= 0) {
                RCLCPP_WARN(driver_->get_logger(), "Predict size failed for w=%d h=%d", frame.width, frame.height);
                encoding.clear();
              } else {
                if (rgb_buffer_.size() < static_cast<size_t>(required)) {
                  rgb_buffer_.resize(required);
                }
                converter_.convert(frame, rgb_buffer_.data());
                mat = cv::Mat{frame.height, frame.width, CV_8UC3, rgb_buffer_.data()};
                encoding = sensor_msgs::image_encodings::RGB8;
              }
            }
          }

          // Display (comment out to avoid Qt threading issues)
          // cv::imshow("frame", mat);
          // cv::waitKey(1);

          // Synchronize ROS messages
          auto stamp = driver_->now();

          if (!encoding.empty() && driver_->count_subscribers(driver_->image_pub_->get_topic_name()) > 0)
          {
            std_msgs::msg::Header header{};
            header.frame_id = downvision_active_ ? frame_id_down_ : frame_id_forward_;
            header.stamp = stamp;
            cv_bridge::CvImage cv_image{header, encoding, mat};
            sensor_msgs::msg::Image sensor_image_msg;
            cv_image.toImageMsg(sensor_image_msg);
            driver_->image_pub_->publish(sensor_image_msg);
          }

          if (driver_->count_subscribers(driver_->camera_info_pub_->get_topic_name()) > 0)
          {
            // Keep CameraInfo dimensions in sync with the current stream
            camera_info_msg_.width = frame.width;
            camera_info_msg_.height = frame.height;
            camera_info_msg_.header.stamp = stamp;
            camera_info_msg_.header.frame_id = downvision_active_ ? frame_id_down_ : frame_id_forward_;
            driver_->camera_info_pub_->publish(camera_info_msg_);
          }
        }

        next += result.num_bytes_consumed;
      }
    }
    catch (std::runtime_error e)
    {
      RCLCPP_ERROR(driver_->get_logger(), e.what());
    }
  }

} // namespace tello_driver
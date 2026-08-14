// Section: ROS 2 external OpenCV camera publisher implementation.

// Section: Standard library headers for durations, exceptions, strings, and smart pointers.
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

// Section: OpenCV camera capture headers.
#include <opencv2/opencv.hpp>

// Section: ROS 2 image conversion and node headers.
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

// Section: ROS 2 node that publishes frames from one external OpenCV camera.
class ExternalCameraNode final : public rclcpp::Node
{
public:
  // Special function: initialize parameters, publishers, camera capture, and the publishing timer.
  ExternalCameraNode()
  : Node("external_camera_node")
  {
    // Section: Runtime parameters for selecting and configuring the camera.
    this->declare_parameter<std::string>("camera_device", "");
    this->declare_parameter<int>("camera_index", 0);
    this->declare_parameter<std::string>("backend", "v4l2");
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<double>("fps", 30.0);

    // Section: Runtime parameters for ROS topic naming and frame metadata.
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    this->declare_parameter<std::string>("frame_id", "external_camera_link");
    this->declare_parameter<bool>("publish_camera_info", true);

    // Section: Cache parameter values used for camera configuration and timer callbacks.
    frame_id_ = this->get_parameter("frame_id").as_string();
    image_topic_ = this->get_parameter("image_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    publish_camera_info_ = this->get_parameter("publish_camera_info").as_bool();
    requested_width_ = this->get_parameter("width").as_int();
    requested_height_ = this->get_parameter("height").as_int();
    requested_fps_ = std::max(1.0, this->get_parameter("fps").as_double());

    // Section: Create image and camera-info publishers with the Python node's queue depth.
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);

    // Section: Open, configure, and validate the selected OpenCV camera source.
    open_camera();
    configure_camera();
    verify_camera_opened();

    // Section: Create a periodic callback using the requested frame rate.
    const auto period = std::chrono::duration<double>(1.0 / requested_fps_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ExternalCameraNode::publish_frame, this));

    // Section: Report the configured output to the operator.
    RCLCPP_INFO(
      this->get_logger(), "Publishing external camera frames on %s at %dx%d@%.1ffps",
      image_topic_.c_str(), requested_width_, requested_height_, requested_fps_);
  }

  // Special function: release the OpenCV camera handle before the ROS node is destroyed.
  ~ExternalCameraNode() override
  {
    // Section: Release only an opened capture to make shutdown safe after failed initialization.
    if (capture_.isOpened()) {
      capture_.release();
    }
  }

private:
  // Special function: translate the friendly backend parameter to an OpenCV backend identifier.
  int backend_id() const
  {
    // Section: Use V4L2 only when it was explicitly requested.
    return this->get_parameter("backend").as_string() == "v4l2" ? cv::CAP_V4L2 : cv::CAP_ANY;
  }

  // Special function: open the camera using a device path first, then an index fallback.
  void open_camera()
  {
    // Section: Prefer the explicit device path when the parameter is non-empty.
    const std::string device = this->get_parameter("camera_device").as_string();
    if (!device.empty()) {
      capture_.open(device, backend_id());
      return;
    }

    // Section: Fall back to the configured numeric OpenCV camera index.
    capture_.open(this->get_parameter("camera_index").as_int(), backend_id());
  }

  // Special function: request capture dimensions and frame rate from OpenCV.
  void configure_camera()
  {
    // Section: Request each property only when its configured value is positive.
    if (requested_width_ > 0) {
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, requested_width_);
    }
    if (requested_height_ > 0) {
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, requested_height_);
    }
    if (requested_fps_ > 0.0) {
      capture_.set(cv::CAP_PROP_FPS, requested_fps_);
    }
  }

  // Special function: fail fast with the selected source when OpenCV cannot open the camera.
  void verify_camera_opened() const
  {
    // Section: Continue initialization only with a usable OpenCV capture handle.
    if (capture_.isOpened()) {
      return;
    }

    // Section: Include the same source choice used by the Python node in the error message.
    const std::string device = this->get_parameter("camera_device").as_string();
    const std::string source = device.empty() ?
      std::to_string(this->get_parameter("camera_index").as_int()) : device;
    throw std::runtime_error("Unable to open external camera source: " + source);
  }

  // Special function: create minimal uncalibrated CameraInfo that matches a published image.
  sensor_msgs::msg::CameraInfo camera_info_message(const sensor_msgs::msg::Image & image) const
  {
    // Section: Mirror the image header and actual image dimensions.
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = image.header;
    camera_info.width = image.width;
    camera_info.height = image.height;

    // Section: Leave calibration and distortion information empty, matching the Python node.
    camera_info.distortion_model = "";
    return camera_info;
  }

  // Special function: capture, timestamp, convert, and publish one BGR image frame.
  void publish_frame()
  {
    // Section: Read one frame and tolerate transient USB or camera driver failures.
    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to read frame from external camera");
      return;
    }

    // Section: Convert OpenCV's BGR frame into a stamped ROS Image message.
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = frame_id_;
    const auto image = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    // Section: Publish the image and optional matching camera metadata.
    image_publisher_->publish(*image);
    if (publish_camera_info_) {
      camera_info_publisher_->publish(camera_info_message(*image));
    }
  }

  // Section: Cached ROS parameters and OpenCV camera capture state.
  std::string frame_id_;
  std::string image_topic_;
  std::string camera_info_topic_;
  bool publish_camera_info_{true};
  int requested_width_{640};
  int requested_height_{480};
  double requested_fps_{30.0};
  cv::VideoCapture capture_;

  // Section: ROS publishers and periodic frame callback timer.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Special function: initialize ROS, spin the external camera node, and cleanly shut down.
int main(int argc, char * argv[])
{
  // Section: Initialize the ROS 2 runtime before creating the node.
  rclcpp::init(argc, argv);

  // Section: Spin the node until ROS shutdown, then release resources deterministically.
  try {
    rclcpp::spin(std::make_shared<ExternalCameraNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("external_camera_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  // Section: Shut down the ROS 2 runtime after normal executor completion.
  rclcpp::shutdown();
  return 0;
}

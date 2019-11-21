#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <signal.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <librealsense2/rs.hpp>

#include "common.hpp"
#include "LinearInterpolation.hpp"
#include "rs.hpp"

#define Vector3StampedMsg geometry_msgs::Vector3Stamped
#define ImuMsg sensor_msgs::Imu

//

//d435i用的数据结构
struct intel_d435i_node_t {
  image_transport::Publisher cam0_pub;
  image_transport::Publisher cam1_pub;
  image_transport::Publisher camrgb_pub;
  image_transport::Publisher cam0_depth_pub;

  ros::Publisher gyro0_pub;
  ros::Publisher accel0_pub;
  ros::Publisher imu0_pub;

  bool global_time;

  rs_cam_module_config_t cam_config;//立体相机的配置文件
  rs_imu_module_config_t imu_config;//imu的配置文件

  intel_d435i_node_t(int argc, char **argv) {
    std::string node_name;
    for (int i = 1; i < argc; i++) {
      std::string arg(argv[i]);

      if (arg.find("__name:=") != std::string::npos) {
        node_name = arg.substr(8);
      }
    }

    ros::init(argc, argv, argv[0]);
    ros::NodeHandle nh;

    const std::string ns = "realsense";
    ROS_GET_PARAM(ns + "/global_time", global_time);//默认为true
    ROS_GET_PARAM(ns + "/sync_size", cam_config.sync_size);//30
    ROS_GET_PARAM(ns + "/enable_emitter", cam_config.enable_emitter);//false
    ROS_GET_PARAM(ns + "/frame_rate", cam_config.frame_rate);//30
    ROS_GET_PARAM(ns + "/stereo_format", cam_config.stereo_format);
    ROS_GET_PARAM(ns + "/depth_format", cam_config.depth_format);
    ROS_GET_PARAM(ns + "/rgb_format", cam_config.rgb_format);
    ROS_GET_PARAM(ns + "/width", cam_config.width);
    ROS_GET_PARAM(ns + "/height", cam_config.height);
    ROS_GET_PARAM(ns + "/exposure", cam_config.exposure);
    ROS_GET_PARAM(ns + "/auto_exposure", cam_config.auto_exposure);
    ROS_GET_PARAM(ns + "/auto_white_balance", cam_config.auto_white_balance);


      //定义一下立体相机的话题
    image_transport::ImageTransport it(nh);
    cam0_pub = it.advertise("/camera/fisheye1/image_raw", 1);
    cam1_pub = it.advertise("/camera/fisheye2/image_raw", 1);
    cam0_depth_pub = it.advertise("/camera/depth/image_raw", 1);
    camrgb_pub = it.advertise("/camera/color/image_raw", 1);
    //imu话题
    gyro0_pub = nh.advertise<Vector3StampedMsg>("camera/gyro", 1);
    accel0_pub = nh.advertise<Vector3StampedMsg>("camera/accel", 1);
    imu0_pub = nh.advertise<ImuMsg>("/camera/imu", 1);
  }
};

static void rgb_handler(const rs2::frameset &fs,
                           const intel_d435i_node_t &node,
                           const bool debug = false) {
    //rgb相机接收
    if (fs.size() != 1) {
        return;
    }
    const auto rgb_img = fs.get_color_frame();
    const int width = rgb_img.get_width();
    const int height = rgb_img.get_height();
    //创建消息
    const auto rgb_msg = create_image_msg(rgb_img, "/camera/color/image_raw",image_type::rgb);

    node.camrgb_pub.publish(rgb_msg);
}

static void stereo_handler(const rs2::frameset &fs,
                           const intel_d435i_node_t &node,
                           const bool debug = false) {
  if (fs.size() != 3) {
    return;
  }

  //立体红外相机接收
  const auto ir_left = fs.get_infrared_frame(1);
  const auto ir_right = fs.get_infrared_frame(2);
  const auto depth = fs.get_depth_frame();
  const int width = ir_left.get_width();
  const int height = ir_left.get_height();
  cv::Mat frame_left = frame2cvmat(ir_left, width, height,image_type::ir);
  cv::Mat frame_right = frame2cvmat(ir_right, width, height,image_type::ir);

  //创建消息
  const auto cam0_msg = create_image_msg(ir_left, "/camera/fisheye1/image_raw",image_type::ir);
  const auto cam1_msg = create_image_msg(ir_right, "/camera/fisheye2/image_raw",image_type::ir);
  const auto depth_msg = create_image_msg(depth, "/camera/depth/image_raw",image_type::depth);


  node.cam0_pub.publish(cam0_msg);
  node.cam1_pub.publish(cam1_msg);
  node.cam0_depth_pub.publish(depth_msg);

  if (debug) {
    debug_imshow(frame_left, frame_right);
  }
}

static void IMU_handler(const rs2::frame &f, const intel_d435i_node_t &node,
                        LinearInterpolation_buf_t &lerp_buf)
{
  const auto mf = f.as<rs2::motion_frame>();

  if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
    const auto msg = create_vec3_msg(mf, "accel0");
    node.accel0_pub.publish(msg);
    lerp_buf.addAccel(msg);//添加加速度数据

  } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
    const auto msg = create_vec3_msg(mf, "gyro0");
    node.gyro0_pub.publish(msg);
    lerp_buf.addGyro(msg);//添加陀螺仪数据
  }

  if (lerp_buf.ready())
  {
    lerp_buf.interpolate();//将低频加速度数据对高频陀螺仪数据进行插值
    lerp_buf.publishIMUMessages(node.imu0_pub);
  }
}

int main(int argc, char **argv) {
  try {
    intel_d435i_node_t node(argc, argv);

    //启动一下D435i吧
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    rs2::device device = rs2_connect();//连接设备

    rs_IMU_module_t IMU{device, node.imu_config};//imu的配置文件往往
    rs_stereo_module_t stereo{device, node.cam_config};//双目的配置文件
      rs_cam_module_config_t rgb_config = node.cam_config;//立体相机的配置文件

      rs_RGB_Camera_t rgb{device, rgb_config};
    //处理一下imu的数据
    LinearInterpolation_buf_t lIP_buf;
    //处理imu
    std::thread IMU_thread([&]() {
      while (true) {
        const auto f = IMU.waitForFrame();//处理所有的imu数据，进行加速度计和陀螺仪之前真正的同步的插值
          IMU_handler(f, node, lIP_buf);
      }
    });

    //处理立体红外相机
    std::thread stereo_thread([&]() {
      while (true) {
        const auto fs = stereo.waitForFrame();
        stereo_handler(fs, node);
      }
    });

    //处理rgb相机
    std::thread rgb_thread([&]()
    {
        while (true) {
            const auto fs = rgb.waitForFrame();
            rgb_handler(fs, node);
        }
    });

    IMU_thread.join();
    stereo_thread.join();
    rgb_thread.join();
  } catch (const rs2::error &e) {
    FATAL("[RealSense Exception]: %s", e.what());
  }

  return 0;
}

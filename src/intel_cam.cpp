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
struct intel_cam_node_t {
  image_transport::Publisher cam0_pub;
  image_transport::Publisher cam1_pub;
  image_transport::Publisher camrgb_pub;
  image_transport::Publisher cam0_depth_pub;

  ros::Publisher gyro0_pub;
  ros::Publisher accel0_pub;
  ros::Publisher imu0_pub;

  bool global_time;
  sensor_type CamType;

  rs_cam_module_config_t cam_config;//立体相机的配置文件
  rs_imu_module_config_t imu_config;//imu的配置文件

//  intel_cam_node_t(int argc, char **argv)
    intel_cam_node_t(ros::NodeHandle & nh,int camnum)
  {
//    std::string node_name;
//    for (int i = 1; i < argc; i++) {
//      std::string arg(argv[i]);
//
//      if (arg.find("__name:=") != std::string::npos) {
//        node_name = arg.substr(8);
//      }
//    }
//
//    ros::init(argc, argv, argv[0]);
//    ros::NodeHandle nh;

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
    int type_index;
    ROS_GET_PARAM(ns + "/cam_type", type_index);
    CamType = (sensor_type)type_index;


      //定义一下立体相机的话题
    image_transport::ImageTransport it(nh);
    cam0_pub = it.advertise("/camera"+ std::to_string(camnum) + "/fisheye1/image_raw", 1);
    cam1_pub = it.advertise("/camera"+ std::to_string(camnum) + "/fisheye2/image_raw", 1);
    cam0_depth_pub = it.advertise("/camera" + std::to_string(camnum) +"/depth/image_raw", 1);
    camrgb_pub = it.advertise("/camera" + std::to_string(camnum) + "/color/image_raw", 1);
    //imu话题
    gyro0_pub = nh.advertise<Vector3StampedMsg>("camera" + std::to_string(camnum) + "/gyro", 1);
    accel0_pub = nh.advertise<Vector3StampedMsg>("camera" + std::to_string(camnum) + "/accel", 1);
    imu0_pub = nh.advertise<ImuMsg>("/camera" +  std::to_string(camnum)  + "/imu", 1);
  }
};

static void rgb_handler(const rs2::frameset &fs,
                           const intel_cam_node_t &node,
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

static void T265_stereo_handler(const rs2::frame &fs,
                           const intel_cam_node_t &node) {

    if(fs.get_profile().stream_index() == 1)
    {
        const auto fisheye_left = fs.as<rs2::video_frame>();
        const int width = fisheye_left.get_width();
        const int height = fisheye_left.get_height();
        cv::Mat frame_left = frame2cvmat(fisheye_left, width, height, image_type::ir);
        const auto cam0_msg = create_image_msg(fisheye_left, "/camera/fisheye1/image_raw", image_type::ir);
        node.cam0_pub.publish(cam0_msg);
    } else if(fs.get_profile().stream_index() == 2)
    {
        const auto fisheye_right = fs.as<rs2::video_frame>();
        const int width = fisheye_right.get_width();
        const int height = fisheye_right.get_height();
        cv::Mat frame_right = frame2cvmat(fisheye_right, width, height, image_type::ir);
        const auto cam1_msg = create_image_msg(fisheye_right, "/camera/fisheye2/image_raw", image_type::ir);
        node.cam1_pub.publish(cam1_msg);
    }

//    //立体红外相机接收
////    const auto fisheye_left = fs.get_fisheye_frame(1);
////    const auto fisheye_right = fs.get_fisheye_frame(2);
//    const int width = fisheye_left.get_width();
//    const int height = fisheye_left.get_height();
//    cv::Mat frame_left = frame2cvmat(fisheye_left, width, height, image_type::ir);
////    cv::Mat frame_right = frame2cvmat(fisheye_right, width, height, image_type::ir);
//
//    //创建消息
//    const auto cam0_msg = create_image_msg(fisheye_left, "/camera/fisheye1/image_raw", image_type::ir);
////    const auto cam1_msg = create_image_msg(fisheye_right, "/camera/fisheye2/image_raw", image_type::ir);
//
//    node.cam0_pub.publish(cam0_msg);
////    node.cam1_pub.publish(cam1_msg);

}

static void stereo_handler(const rs2::frameset &fs,
                           const intel_cam_node_t &node,
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

static void IMU_handler(const rs2::frame &f, const intel_cam_node_t &node,
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

void ProcessSingleD435iCam(const rs2::device &device, const intel_cam_node_t &node)
{
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
}

void ProcessSingleD435Cam(const rs2::device &device, const intel_cam_node_t &node)
{
    rs_stereo_module_t stereo{device, node.cam_config};//双目的配置文件
    rs_cam_module_config_t rgb_config = node.cam_config;//立体相机的配置文件

    rs_RGB_Camera_t rgb{device, rgb_config};

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
    stereo_thread.join();
    rgb_thread.join();
}

void ProcessSingleT265Cam(const rs2::device &device, const intel_cam_node_t &node)
{

//    rs_T265_IMU_module_t IMU{device};//imu的配置文件往往
////处理一下imu的数据
    LinearInterpolation_buf_t lIP_buf;

    ////相机
    rs_T265_cam_module_config_t t265config;
    t265config.global_time = node.cam_config.global_time;
    t265config.sync_size = node.cam_config.sync_size;
    t265config.frame_rate = node.cam_config.frame_rate;
    t265config.stereo_format = node.cam_config.stereo_format;
    t265config.exposure = node.cam_config.exposure;
    rs_T265_cam_module_t stereo_fisheye{device, t265config};//双目的配置文件
//    std::thread IMU_thread([&]() {
//        while (true) {
//            const auto f = IMU.waitForFrame();//处理所有的imu数据，进行加速度计和陀螺仪之前真正的同步的插值
//            IMU_handler(f, node, lIP_buf);
//        }
//    });

    std::thread stereo_thread([&]()
    {
        while (true) {
            const auto fs = stereo_fisheye.waitForFrame();
            if(fs.get_profile().stream_type() == rs2_stream::RS2_STREAM_FISHEYE)
            {
                T265_stereo_handler(fs, node);

            } else
            {
                IMU_handler(fs, node, lIP_buf);
            }
        }
    });

//    IMU_thread.join();
    stereo_thread.join();

}



int main(int argc, char **argv)
{
    try {
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
        int type_index;
        ROS_GET_PARAM(ns + "/cam_type", type_index);
        sensor_type CamType = (sensor_type)type_index;
        //启动一下D435i吧
        rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
        rs2::device_list devices = rs2_connect();

        if(CamType == sensor_type::T265)//多相机模式
        {
            int Num_Cam = devices.size();
            std::vector<intel_cam_node_t> nodes;
            nodes.clear();
            for (int camid = 0; camid < Num_Cam; ++camid)
            {
                auto sn = devices[camid].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                const std::string numid = std::to_string(905312111244);
                if(numid == sn)//后相机
                {
                    intel_cam_node_t node(nh,1);
                    nodes.push_back(node);
                }
                else
                {
                    intel_cam_node_t node(nh,2);//前
                    nodes.push_back(node);
                }
            }
            std::thread Cam_thread[Num_Cam];

            for (int i = 0; i < Num_Cam; ++i)//905312111244
            {
                Cam_thread[i] = std::thread(ProcessSingleT265Cam,devices[i],std::ref(nodes[i]));
            }
            for (int j = 0; j < Num_Cam; ++j)
            {
                Cam_thread[j].join();
            }

        }
        else if(CamType == sensor_type::D435i)//只用单个相机吧
        {
            int Num_Cam = devices.size();
            std::vector<intel_cam_node_t> nodes;
            nodes.clear();
            for (int camid = 0; camid < Num_Cam; ++camid)
            {
                intel_cam_node_t node(nh,camid);
                nodes.push_back(node);
            }
            std::thread Cam_thread[Num_Cam];
            for (int i = 0; i < Num_Cam; ++i)
            {
                Cam_thread[i] = std::thread(ProcessSingleD435iCam,devices[i],std::ref(nodes[i]));
            }
            for (int j = 0; j < Num_Cam; ++j)
            {
                Cam_thread[j].join();
            }
        }
        else if(CamType == sensor_type::D435)
        {
            int Num_Cam = devices.size();
            std::vector<intel_cam_node_t> nodes;
            nodes.clear();
            for (int camid = 0; camid < Num_Cam; ++camid)
            {
                intel_cam_node_t node(nh,camid);
                nodes.push_back(node);
            }
            std::thread Cam_thread[Num_Cam];
            for (int i = 0; i < Num_Cam; ++i)
            {
                Cam_thread[i] = std::thread(ProcessSingleD435Cam,devices[i],std::ref(nodes[i]));
            }
            for (int j = 0; j < Num_Cam; ++j)
            {
                Cam_thread[j].join();
            }
        }
    } catch (const rs2::error &e) {
        FATAL("[RealSense Exception]: %s", e.what());
    }
  return 0;
}

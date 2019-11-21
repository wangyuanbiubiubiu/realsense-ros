#pragma once
#include <iostream>
#include <string.h>

#include <librealsense2/rs.hpp>

#include "common.hpp"


rs2::device rs2_connect() {//连接一下设备
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  rs2::device device;
  if (devices.size() == 0) {
    FATAL("No device connected, please connect a RealSense device");
  }

  return devices[0];
}

void rs2_list_sensors(const int device_idx = 0) {
  //连接一下设备
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  rs2::device device;
  if (devices.size() == 0) {
    FATAL("No device connected, please connect a RealSense device");
  } else {
    device = devices[device_idx]; //目前只去针对单设备的情况
  }

  printf("Sensors:\n");
  for (const auto &query_sensor : device.query_sensors()) {
    const auto sensor_name = query_sensor.get_info(RS2_CAMERA_INFO_NAME);
    printf("  - %s\n", sensor_name);
  }
}

int rs2_get_sensors(const rs2::device &device, const std::string &target,
                    rs2::sensor &sensor) {
  for (const auto &query_sensor : device.query_sensors()) {
    const auto sensor_name = query_sensor.get_info(RS2_CAMERA_INFO_NAME);
    if (strcmp(sensor_name, target.c_str()) == 0) {
      sensor = query_sensor;
      return 0;
    }
  }

  return -1;
}

struct rs_imu_module_config_t {
  bool global_time = true;
  int accel_hz = 250;
  int gyro_hz = 400;
  unsigned int fq_size = 10;
};

class rs_IMU_module_t {
  const rs2::device &device_;
  rs2::frame_queue fq_;
  rs2::sensor sensor_;
  rs2::stream_profile accel_profile_;//加速度计的配置文件
  rs2::stream_profile gyro_profile_;//陀螺仪的配置文件
  rs_imu_module_config_t config_;

public:
  rs_IMU_module_t(const rs2::device &device)
      : device_{device}, fq_{config_.fq_size} {
    setup(config_.accel_hz, config_.gyro_hz);
  }

  rs_IMU_module_t(const rs2::device &device,
                  const rs_imu_module_config_t &config)
      : device_{device}, fq_{config.fq_size}, config_{config} {//初始化imu相关参数
    setup(config_.accel_hz, config_.gyro_hz);//设置imu的相关配置
  }

  ~rs_IMU_module_t() {
    sensor_.stop();
    sensor_.close();
  }

  void listStreamProfiles() {
    // Go through Stream profiles
    std::cout << "Motion module stream profiles:" << std::endl;
    const auto stream_profiles = sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles) {
      const auto stream_name = stream_profile.stream_name();
      const auto stream_rate = stream_profile.fps();
      std::cout << " - " << stream_name << " " << stream_rate << " hz ";
      std::cout << std::endl;
    }
  }
//分别按照给定的加速度计和陀螺仪的频率来接收话题
  void setStreamProfiles(const int accel_hz, const int gyro_hz) {
    bool accel_ok = false;
    bool gyro_ok = false;

    // Go through Stream profiles
    const auto stream_profiles = sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles) {
      const auto stream_name = stream_profile.stream_name();//当前消息流
      const auto stream_rate = stream_profile.fps();

      if (stream_name == "Accel" && stream_rate == accel_hz) {
        accel_profile_ = stream_profile;
        accel_ok = true;
      }

      if (stream_name == "Gyro" && stream_rate == gyro_hz) {
        gyro_profile_ = stream_profile;
        gyro_ok = true;
      }
    }

    if (accel_ok == false) {
      FATAL("加速计的频率有点问题,%d Hz", accel_hz);
    }
    if (gyro_ok == false) {
      FATAL("陀螺仪的频率有点问题,%d Hz", gyro_hz);
    }
  }

  void setup(const int accel_hz = 250, const int gyro_hz = 400) {//设置一下消息流
    if (rs2_get_sensors(device_, "Motion Module", sensor_) != 0) {
      FATAL("This RealSense device does not have a [Motion Module]");
    }

    //分别按照给定的加速度计和陀螺仪的频率来接收话题
    setStreamProfiles(accel_hz, gyro_hz);

    //是否需要全局时间
    sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);

    //启动这两个
    sensor_.open({accel_profile_, gyro_profile_});
    sensor_.start(fq_);
  }

  rs2::frame waitForFrame() { return fq_.wait_for_frame(); }//等待imu的消息
};


struct rs_cam_module_config_t {
  bool global_time = true;
  int sync_size = 30;
  bool enable_emitter = false;

  int frame_rate = 30;
  std::string stereo_format = "Y8";
  std::string depth_format = "Z16";
  std::string rgb_format = "RGB8";

  int width = 640;
  int height = 480;
  double exposure = 0.0f;//曝光时间
  bool auto_exposure = true;
  bool auto_white_balance = true;

};

class rs_stereo_module_t {
  const rs2::device &device_;
  rs2::syncer sync_;
  rs2::sensor stereo_module_sensor_;

    //
  rs2::stream_profile profile_ir1;
  rs2::stream_profile profile_ir2;
  rs2::stream_profile profile_depth;
  bool profile_ir1_set_ = false;
  bool profile_ir2_set_ = false;
  bool profile_depth_set = false;
  rs_cam_module_config_t config_;

public:
  rs_stereo_module_t(const rs2::device &device)
      : device_{device}, sync_{config_.sync_size} {
    setup();
  }

  rs_stereo_module_t(const rs2::device &device,
                     const rs_cam_module_config_t &config)
      : device_{device}, sync_{config.sync_size}, config_{config} {
    setup();
  }

  ~rs_stereo_module_t() {
    stereo_module_sensor_.stop();
    stereo_module_sensor_.close();
  }

  void listStreamProfiles() {
    // Go through Stream profiles
    std::cout << "Stereo module stream profiles:" << std::endl;
    const auto stream_profiles = stereo_module_sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles) {
      const auto stream_name = stream_profile.stream_name();
      const auto stream_rate = stream_profile.fps();
      const auto format = rs2_format_to_string(stream_profile.format());
      const auto vp = stream_profile.as<rs2::video_stream_profile>();
      printf("- %s [%d hz] [%s] [%dx%d] \n", stream_name.c_str(), stream_rate,
             format, vp.width(), vp.height());
    }
  }

void setStreamProfile()
{
    const auto stream_profiles = stereo_module_sensor_.get_stream_profiles();
    for (const auto &stream_profile : stream_profiles)
    {
        const auto name = stream_profile.stream_name();
        const auto rate = stream_profile.fps();
        const auto format = rs2_format_to_string(stream_profile.format());
        const auto vp = stream_profile.as<rs2::video_stream_profile>();
        const int width = vp.width();
        const int height = vp.height();

        const bool rate_ok = (config_.frame_rate == rate);
        const bool stereo_format_ok = (config_.stereo_format == format);
        const bool depth_format_ok = (config_.depth_format == format);

        const bool width_ok = (config_.width == width);
        const bool height_ok = (config_.height == height);
        const bool res_ok = (width_ok && height_ok);

        if (rate_ok && stereo_format_ok && res_ok)
        {
            if ("Infrared 1" == name) {
                profile_ir1 = stream_profile;
                profile_ir1_set_ = true;
            } else if ("Infrared 2" == name) {
                profile_ir2 = stream_profile;
                profile_ir2_set_ = true;
            }
        } else if (rate_ok && depth_format_ok && res_ok)
        {
            if ("Depth" == name)
            {
                profile_depth = stream_profile;
                profile_depth_set = true;
            }
        }
    }

    if (profile_ir1_set_ == false && profile_ir2_set_ == false) {
        FATAL("Failed to get stereo module stream profile!");
    }
    if (profile_depth == false) {
        FATAL("Failed to get depth module stream profile!");
    }
}

  void setup()
  {
      if (rs2_get_sensors(device_, "Stereo Module", stereo_module_sensor_) != 0) {
          FATAL("This RealSense device does not have a [Stereo Module]");
      }
      //设置一下双目的参数吧
      setStreamProfile();

      //先把激光发射器给关上吧
      stereo_module_sensor_.set_option(RS2_OPTION_EMITTER_ENABLED, config_.enable_emitter);

      stereo_module_sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);

      stereo_module_sensor_.set_option(RS2_OPTION_EXPOSURE, config_.exposure);

      stereo_module_sensor_.open({profile_ir1, profile_ir2, profile_depth});
      stereo_module_sensor_.start(sync_);
  }

  rs2::frameset waitForFrame() { return sync_.wait_for_frames(10000); }
};

class rs_RGB_Camera_t {
    const rs2::device &device_;
    rs2::syncer fq_;
    rs2::sensor RGB_Camera_sensor_;

    rs2::stream_profile profile_rgb;
    bool profile_rgb_set_ = false;

    rs_cam_module_config_t config_;

public:
    rs_RGB_Camera_t(const rs2::device &device)
            : device_{device}, fq_{10}
            {
        setup();
    }

    rs_RGB_Camera_t(const rs2::device &device,
                       const rs_cam_module_config_t &config)
            : device_{device}, fq_{10}, config_{config} {
        setup();
    }

    ~rs_RGB_Camera_t() {
        RGB_Camera_sensor_.stop();
        RGB_Camera_sensor_.close();
    }

    void setStreamProfile()
    {
        const auto stream_profiles = RGB_Camera_sensor_.get_stream_profiles();
        for (const auto &stream_profile : stream_profiles)
        {
            const auto name = stream_profile.stream_name();
            const auto rate = stream_profile.fps();
            const auto format = rs2_format_to_string(stream_profile.format());
            const auto vp = stream_profile.as<rs2::video_stream_profile>();
            const int width = vp.width();
            const int height = vp.height();

            const bool rate_ok = (config_.frame_rate == rate);
            const bool rgb_format_ok = (config_.rgb_format == format);

            const bool width_ok = (config_.width == width);
            const bool height_ok = (config_.height == height);
            const bool res_ok = (width_ok && height_ok);

            if (rate_ok && rgb_format_ok && res_ok)
            {
                if ("Color" == name) {
                    profile_rgb = stream_profile;
                    profile_rgb_set_ = true;
                }
            }
        }

        if (profile_rgb_set_ == false) {
            FATAL("Failed to get rgb cam stream profile!");
        }

    }

    void setup()
    {
        if (rs2_get_sensors(device_, "RGB Camera", RGB_Camera_sensor_) != 0) {
            FATAL("This RealSense device does not have a [RGB Camera]");
        }
        //设置一下RGB的参数吧
        setStreamProfile();


        RGB_Camera_sensor_.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, config_.global_time);

        RGB_Camera_sensor_.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, config_.auto_exposure);
        RGB_Camera_sensor_.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,config_.auto_white_balance);

        RGB_Camera_sensor_.open({profile_rgb});
        RGB_Camera_sensor_.start(fq_);
    }

    rs2::frameset waitForFrame() { return fq_.wait_for_frames(10000); }
};
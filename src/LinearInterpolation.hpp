#pragma once
#include "common.hpp"

template <typename T> T LinearInterpolation(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

class LinearInterpolation_buf_t {
  std::deque<std::string> buf_type_;
  std::deque<double> buf_ts_;
  std::deque<Eigen::Vector3d> buf_data_;

  std::deque<double> LinearInterpolation_gyro_ts_;
  std::deque<Eigen::Vector3d> LinearInterpolation_gyro_data_;
  std::deque<double> LinearInterpolation_accel_ts_;
  std::deque<Eigen::Vector3d> LinearInterpolation_accel_data_;

public:
  LinearInterpolation_buf_t() {}

  bool ready() {
    if (buf_ts_.size() >= 3 && buf_type_.back() == "A") {
      return true;
    }
    return false;
  }

  void addAccel(const geometry_msgs::Vector3Stamped &msg) {
    buf_type_.push_back("A");//好区分加速度和陀螺仪
    buf_ts_.push_back(msg.header.stamp.toSec());
    buf_data_.emplace_back(msg.vector.x, msg.vector.y, msg.vector.z);
  }

  void addGyro(const geometry_msgs::Vector3Stamped &msg) {
    if (buf_type_.size() && buf_type_.front() == "A") {
      buf_type_.push_back("G");
      buf_ts_.push_back(msg.header.stamp.toSec());
      buf_data_.emplace_back(msg.vector.x, msg.vector.y, msg.vector.z);
    }
  }

  void print() {
    for (size_t i = 0; i < buf_ts_.size(); i++) {
      const double ts = buf_ts_.at(i);
      const std::string dtype = buf_type_.at(i);
      const Eigen::Vector3d data = buf_data_.at(i);
      const double x = data(0);
      const double y = data(1);
      const double z = data(1);
      printf("[%.6f] - [%s] - (%.2f, %.2f, %.2f)\n", ts, dtype.c_str(), x, y,
             z);
    }
  }

  void interpolate() {
    //插值数据
    double t0 = 0;
    Eigen::Vector3d d0;
    double t1 = 0;
    Eigen::Vector3d d1;
    bool t0_set = false;

    std::deque<double> lerp_ts;
    std::deque<Eigen::Vector3d> lerp_data;

    double ts = 0.0;
    std::string dtype;
    Eigen::Vector3d data;

    while (buf_ts_.size())
    {
      //时间戳，不一定是加速度计还是陀螺仪的
      ts = buf_ts_.front();
      buf_ts_.pop_front();

      //数据类型，这样就能指明加速度计的还是陀螺仪的
      dtype = buf_type_.front();
      buf_type_.pop_front();

      //数据，不一定是加速度计还是陀螺仪的
      data = buf_data_.front();
      buf_data_.pop_front();

      //开始插值，用低频的加速度数据去插值高频的陀螺仪数据，如果是加速度计的话，先接收一下数据
      if (t0_set == false && dtype == "A") {
          //加速度计的数据
        t0 = ts;
        d0 = data;
        t0_set = true;

      } else if (t0_set && dtype == "A")
      {
          //加速度计的数据
        t1 = ts;
        d1 = data;

        while (lerp_ts.size())
        {
          const double lts = lerp_ts.front();//要插值的时间戳，这个肯定是陀螺仪的
          const Eigen::Vector3d ldata = lerp_data.front();//陀螺仪的数据
          const double dt = t1 - t0;
          const double alpha = (lts - t0) / dt;

          LinearInterpolation_accel_ts_.push_back(lts);
          LinearInterpolation_accel_data_.push_back(LinearInterpolation(d0, d1, alpha));//将加速度数据进行插值

          LinearInterpolation_gyro_ts_.push_back(lts);
          LinearInterpolation_gyro_data_.push_back(ldata);

          lerp_ts.pop_front();
          lerp_data.pop_front();
        }

        t0 = t1;//置换一下前一次的imu的数据
        d0 = d1;

      } else if (t0_set && ts >= t0 && dtype == "G")
      {
          //如果是陀螺仪的话，就把陀螺仪的数据加入
        lerp_ts.push_back(ts);
        lerp_data.push_back(data);
      }
    }

    buf_ts_.push_back(ts);
    buf_type_.push_back(dtype);
    buf_data_.push_back(data);
  }

  void publishIMUMessages(const ros::Publisher &imu_pub) {
    while (LinearInterpolation_gyro_ts_.size()) {
      // Timestamp
      const auto ts = LinearInterpolation_gyro_ts_.front();
      LinearInterpolation_gyro_ts_.pop_front();
      LinearInterpolation_accel_ts_.pop_front();

      // Accel
      const auto accel = LinearInterpolation_accel_data_.front();
      LinearInterpolation_accel_data_.pop_front();

      // Gyro
      const auto gyro = LinearInterpolation_gyro_data_.front();
      LinearInterpolation_gyro_data_.pop_front();

      // Publish imu messages
      const auto msg = create_imu_msg(ts, gyro, accel);
      imu_pub.publish(msg);
    }

    // Clear
    LinearInterpolation_gyro_ts_.clear();
    LinearInterpolation_gyro_data_.clear();
    LinearInterpolation_accel_ts_.clear();
    LinearInterpolation_accel_data_.clear();
  }
};

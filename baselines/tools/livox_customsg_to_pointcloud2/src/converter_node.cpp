// converter_node.cpp — Livox CustomMsg to sensor_msgs/PointCloud2 converter.
//
// PURPOSE
// ------
// DLIO (and other algorithms) subscribe to sensor_msgs/PointCloud2 and expect
// per-point timestamps for deskewing. Livox sensors in M3DGR bags publish
// livox_ros_driver/CustomMsg which is a proprietary format. This node bridges
// the two.
//
// POINT LAYOUT (output PointCloud2)
// ---------------------------------
// Field       Type       Offset  Description
// x           float32     0      X coordinate (m)
// y           float32     4      Y coordinate (m)
// z           float32     8      Z coordinate (m)
// intensity   float32    12      Reflectivity (from CustomMsg)
// timestamp   float64    16      Absolute time in nanoseconds (double)
//                                = header.timebase + point.offset_time
//
// DLIO auto-detects this as SensorType::LIVOX when it finds a "timestamp"
// field with values > 1e14, then deskews using timestamp * 1e-9 (seconds).
//
// PARAMS (private)
// ----------------
// ~input_topic   (string, default "/livox/avia/lidar")
// ~output_topic  (string, default "/livox/avia/lidar_pc2")
// ~queue_size    (int,    default 100)

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <livox_ros_driver/CustomMsg.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

// Output point: 24 bytes (4+4+4+4+8)
constexpr uint32_t POINT_STEP = 24;

sensor_msgs::PointField makeField(const std::string& name,
                                  uint32_t offset,
                                  uint8_t datatype) {
  sensor_msgs::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = 1;
  return f;
}

class Converter {
 public:
  Converter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), msg_count_(0) {
    const std::string input_topic =
        pnh_.param<std::string>("input_topic", "/livox/avia/lidar");
    output_topic_ =
        pnh_.param<std::string>("output_topic", "/livox/avia/lidar_pc2");
    const int queue_size = pnh_.param<int>("queue_size", 100);

    ROS_INFO("[livox_to_pc2] input  = %s", input_topic.c_str());
    ROS_INFO("[livox_to_pc2] output = %s", output_topic_.c_str());

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_size);
    sub_ = nh_.subscribe<livox_ros_driver::CustomMsg>(
        input_topic, queue_size,
        &Converter::callback, this);
  }

 private:
  void callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    const uint32_t n_points = msg->point_num;
    if (n_points == 0) {
      return;
    }

    sensor_msgs::PointCloud2 pc2;
    pc2.header = msg->header;
    pc2.height = 1;
    pc2.width = n_points;
    pc2.is_bigendian = false;
    pc2.is_dense = true;
    pc2.point_step = POINT_STEP;
    pc2.row_step = POINT_STEP * n_points;

    // Define fields: x, y, z, intensity, timestamp
    pc2.fields.reserve(5);
    pc2.fields.push_back(
        makeField("x", 0, sensor_msgs::PointField::FLOAT32));
    pc2.fields.push_back(
        makeField("y", 4, sensor_msgs::PointField::FLOAT32));
    pc2.fields.push_back(
        makeField("z", 8, sensor_msgs::PointField::FLOAT32));
    pc2.fields.push_back(
        makeField("intensity", 12, sensor_msgs::PointField::FLOAT32));
    pc2.fields.push_back(
        makeField("timestamp", 16, sensor_msgs::PointField::FLOAT64));

    pc2.data.resize(pc2.row_step);

    const uint64_t timebase = msg->timebase;
    uint8_t* dst = pc2.data.data();

    for (uint32_t i = 0; i < n_points; ++i) {
      const auto& pt = msg->points[i];

      const float x = pt.x;
      const float y = pt.y;
      const float z = pt.z;
      const float intensity = static_cast<float>(pt.reflectivity);
      // Absolute timestamp in nanoseconds (as double).
      // DLIO detects LIVOX sensor when timestamp > 1e14.
      const double timestamp =
          static_cast<double>(timebase + pt.offset_time);

      std::memcpy(dst + 0,  &x,         sizeof(float));
      std::memcpy(dst + 4,  &y,         sizeof(float));
      std::memcpy(dst + 8,  &z,         sizeof(float));
      std::memcpy(dst + 12, &intensity, sizeof(float));
      std::memcpy(dst + 16, &timestamp, sizeof(double));

      dst += POINT_STEP;
    }

    pub_.publish(pc2);

    if ((++msg_count_ % 100) == 0) {
      ROS_INFO_THROTTLE(5.0,
          "[livox_to_pc2] converted %lu msgs (%u pts/msg) on %s",
          msg_count_, n_points, output_topic_.c_str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string output_topic_;
  unsigned long msg_count_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_customsg_to_pointcloud2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  Converter node(nh, pnh);
  ros::spin();
  return 0;
}

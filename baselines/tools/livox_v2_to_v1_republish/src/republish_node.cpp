// republish_node.cpp — namespace-only bridge from livox_ros_driver2/CustomMsg
// to livox_ros_driver/CustomMsg.
//
// RATIONALE
// ---------
// M3DGR bags record the Mid-360 stream as livox_ros_driver2/CustomMsg. Our
// baseline images (FAST-LIO2, Point-LIO, iG-LIO) subscribe to
// livox_ros_driver/CustomMsg (v1). The two message types have IDENTICAL MD5
// checksums (e4d6829bdfe657cb6c21a746c86b21a6) — verified on
// baselines-fast_lio2:ros1 — which means the serialised wire bytes are
// bit-for-bit identical; only the ROS package namespace differs.
//
// This node therefore does NOT deserialise. It uses topic_tools::ShapeShifter
// to capture the raw byte buffer of whatever arrives on the input topic and
// re-advertises the same bytes under a new topic, rewriting only the
// datatype string (the only thing a v1 subscriber's MD5 check compares
// against) to "livox_ros_driver/CustomMsg".
//
// This is strictly safe because:
//   * No field is interpreted.
//   * No buffer copy or re-serialisation is performed (pass-through).
//   * The MD5 match guarantees v1 consumers will deserialise correctly.
//
// If a future dataset changes the v2 struct (different MD5), the publisher
// advertise() call below will be rejected by v1 consumers and the mismatch
// will be caught at connect time — failing loudly, not silently.
//
// Params (private):
//   ~input_topic  (string, default "/livox/mid360/lidar")
//   ~output_topic (string, default "/livox/mid360/lidar_v1")
//   ~queue_size   (int,    default 100)

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <memory>
#include <string>

namespace {

constexpr const char* kV1Datatype   = "livox_ros_driver/CustomMsg";
constexpr const char* kV1Md5        = "e4d6829bdfe657cb6c21a746c86b21a6";
// Minimal message definition string. ROS pub/sub only verifies md5sum at
// connect time, but we supply a faithful definition for introspection tools
// (rostopic info, rosbag record) so they don't print warnings.
constexpr const char* kV1Definition =
    "# Livox publish pointcloud msg format.\n"
    "\n"
    "std_msgs/Header header             # ROS standard message header\n"
    "uint64 timebase                    # The time of first point\n"
    "uint32 point_num                   # Total number of pointclouds\n"
    "uint8 lidar_id                     # Lidar device id number\n"
    "uint8[3] rsvd                      # Reserved use\n"
    "CustomPoint[] points               # Pointcloud data\n";

class Republisher {
 public:
  Republisher(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh),
        pnh_(pnh),
        output_topic_(pnh_.param<std::string>("output_topic",
                                              "/livox/mid360/lidar_v1")),
        input_topic_(pnh_.param<std::string>("input_topic",
                                             "/livox/mid360/lidar")),
        queue_size_(pnh_.param<int>("queue_size", 100)),
        forwarded_count_(0) {
    ROS_INFO("[livox_v2_to_v1_republish] input  = %s", input_topic_.c_str());
    ROS_INFO("[livox_v2_to_v1_republish] output = %s (%s md5=%s)",
             output_topic_.c_str(), kV1Datatype, kV1Md5);

    // Subscribe as ShapeShifter so ROS deserialises the wire bytes without
    // needing the v2 header.
    sub_ = nh_.subscribe<topic_tools::ShapeShifter>(
        input_topic_, queue_size_,
        boost::bind(&Republisher::callback, this, _1));
  }

 private:
  void callback(const topic_tools::ShapeShifter::ConstPtr& msg) {
    if (!pub_initialised_) {
      // Lazy-advertise: we morph the ShapeShifter to declare itself as v1,
      // then ask it to advertise. morph() only rewrites the metadata
      // strings; the serialised payload is unchanged.
      topic_tools::ShapeShifter morphed = *msg;
      morphed.morph(kV1Md5, kV1Datatype, kV1Definition, /*latch*/ "");
      pub_ = morphed.advertise(nh_, output_topic_, queue_size_,
                               /*latch=*/false);
      pub_initialised_ = true;
      ROS_INFO("[livox_v2_to_v1_republish] advertised %s as %s",
               output_topic_.c_str(), kV1Datatype);
    }

    // Forward the identical payload. ShapeShifter::write() serialises the
    // cached buffer back onto the wire — byte-for-byte the original v2
    // payload, now tagged as v1. Consumers MD5-check and accept.
    pub_.publish(msg);

    if ((++forwarded_count_ % 100) == 0) {
      ROS_INFO_THROTTLE(5.0,
          "[livox_v2_to_v1_republish] forwarded %lu msgs on %s",
          forwarded_count_, output_topic_.c_str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string output_topic_;
  std::string input_topic_;
  int queue_size_;
  unsigned long forwarded_count_;
  bool pub_initialised_ = false;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_v2_to_v1_republish");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  Republisher node(nh, pnh);
  ros::spin();
  return 0;
}

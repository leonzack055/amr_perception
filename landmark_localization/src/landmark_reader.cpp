#include "landmark_localization/landmark_reader.hpp"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include <filesystem>
#include <stdexcept>

LandmarkReader::LandmarkReader(const std::string & pbstream_file)
: pbstream_file_(pbstream_file) {}

LandmarkReader::~LandmarkReader()
{
}

bool LandmarkReader::readLandmarks()
{
  try {
    // 创建ProtoStreamReader读取pbstream文件
    cartographer::io::ProtoStreamReader reader(pbstream_file_);

    // 创建反序列化器
    cartographer::io::ProtoStreamDeserializer deserializer(&reader);

    // 获取pose graph数据
    cartographer::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();

    landmarks_.clear();

    // 遍历所有的landmark poses
    for (const auto & landmark : pose_graph_proto.landmark_poses()) {
      LandmarkInfo info;
      info.landmark_id = landmark.landmark_id();

      // 提取全局位姿信息
      const auto & global_pose = landmark.global_pose();

      // 位置信息
      info.x = global_pose.translation().x();
      info.y = global_pose.translation().y();
      info.z = global_pose.translation().z();

      // 姿态信息（四元数）
      info.qx = global_pose.rotation().x();
      info.qy = global_pose.rotation().y();
      info.qz = global_pose.rotation().z();
      info.qw = global_pose.rotation().w();

      landmarks_.push_back(info);
    }

    std::cout << "Successfully read " << landmarks_.size()
              << " landmarks from pose graph" << std::endl;

    return true;

  } catch (const std::exception & e) {
    std::cerr << "Error reading pbstream file: " << e.what() << std::endl;
    return false;
  }
}

const std::vector<LandmarkInfo> & LandmarkReader::getLandmarks() const
{
  return landmarks_;
}

void LandmarkReader::printLandmarks() const
{
  std::cout << "\n==========================================" << std::endl;
  std::cout << "Landmarks found in pose graph: " << landmarks_.size() << std::endl;
  std::cout << "==========================================" << std::endl;

  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const auto & landmark = landmarks_[i];
    std::cout << "Landmark #" << i + 1 << std::endl;
    std::cout << "  ID: " << landmark.landmark_id << std::endl;
    std::cout << "  Position: (" << landmark.x << ", "
              << landmark.y << ", " << landmark.z << ")" << std::endl;
    std::cout << "  Orientation: (" << landmark.qx << ", " << landmark.qy
              << ", " << landmark.qz << ", " << landmark.qw << ")" << std::endl;

    std::cout << "------------------------------------------" << std::endl;
  }
}

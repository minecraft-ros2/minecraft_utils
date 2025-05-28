// Copyright 2025 minecraft-ros2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "minecraft_utils_visualization/mob_marker.hpp"

namespace minecraft_utils_visualization
{

MobMarker::MobMarker(rclcpp::NodeOptions options)
: rclcpp::Node("mob_marker", options)
{
  this->declare_parameter("world_frame_id", "world");
  this->declare_parameter("player_frame_id", "player");
  this->declare_parameter("mob_namespace", "mob");
  this->declare_parameter("mob_text_namespace", "mob_text");

  world_frame_id_ = this->get_parameter("world_frame_id").as_string();
  player_frame_id_ = this->get_parameter("player_frame_id").as_string();
  mob_namespace_ = this->get_parameter("mob_namespace").as_string();
  mob_text_namespace_ = this->get_parameter("mob_text_namespace").as_string();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mob_markers", 10);
  mob_sub_ = this->create_subscription<minecraft_msgs::msg::LivingEntityArray>(
    "player/nearby_living_entities", qos,
    std::bind(&MobMarker::mobCallback, this, std::placeholders::_1));
}


void MobMarker::mobCallback(const minecraft_msgs::msg::LivingEntityArray::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      world_frame_id_, player_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  float player_x = transform_stamped.transform.translation.x;
  float player_y = transform_stamped.transform.translation.y;
  float player_z = transform_stamped.transform.translation.z;

  auto player_orientation = transform_stamped.transform.rotation;

  for (size_t i = 0; i < msg->entities.size(); ++i) {
    const auto & entity = msg->entities[i];

    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;

    marker.header.frame_id = player_frame_id_;
    marker.ns = mob_namespace_;
    marker.id = static_cast<int>(entity.id);
    marker.lifetime = rclcpp::Duration(1, 0);

    marker.type = visualization_msgs::msg::Marker::CUBE;
    this->setMarkerColor(entity.category.mob_category, marker);

    marker.scale.x = entity.hit_box.x;
    marker.scale.y = entity.hit_box.y;
    marker.scale.z = entity.hit_box.z;

    marker.pose = entity.pose;

    auto transformed_position = transformToPlayerFrame(
      player_orientation,
      entity.pose.position.x - player_x,
      entity.pose.position.y - player_y,
      entity.pose.position.z - player_z
    );

    marker.pose.position.x = transformed_position.getX();
    marker.pose.position.y = transformed_position.getY();
    marker.pose.position.z = transformed_position.getZ();

    marker_array.markers.push_back(marker);

    this->addNameHealthMarker(
      entity, marker.header, player_x, player_y, player_z,
      player_orientation, marker_array);
  }

  marker_pub_->publish(marker_array);
}


void MobMarker::setMarkerColor(
  const uint8_t mob_category,
  visualization_msgs::msg::Marker & marker)
{
  switch (mob_category) {
    case minecraft_msgs::msg::MobCategory::MONSTER:
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      break;
    case minecraft_msgs::msg::MobCategory::CREATURE:
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      break;
    case minecraft_msgs::msg::MobCategory::AMBIENT:
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 0.8;
      break;
    case minecraft_msgs::msg::MobCategory::AXOLOTLS:
      marker.color.r = 1.0;
      marker.color.g = 0.4;
      marker.color.b = 0.7;
      marker.color.a = 0.8;
      break;
    case minecraft_msgs::msg::MobCategory::UNDERGROUND_WATER_CREATURE:
      marker.color.r = 0.6;
      marker.color.g = 0.3;
      marker.color.b = 0.1;
      marker.color.a = 0.8;
      break;
    case minecraft_msgs::msg::MobCategory::WATER_CREATURE:
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.8;
      break;
    case minecraft_msgs::msg::MobCategory::WATER_AMBIENT:
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 1.0;
      marker.color.a = 0.8;
      break;
    case minecraft_msgs::msg::MobCategory::MISC:
    default:
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.8;
      break;
  }
}


void MobMarker::addNameHealthMarker(
  const minecraft_msgs::msg::LivingEntity & entity,
  const std_msgs::msg::Header & header,
  const float player_x,
  const float player_y,
  const float player_z,
  const geometry_msgs::msg::Quaternion & player_orientation,
  visualization_msgs::msg::MarkerArray & marker_array)
{
  visualization_msgs::msg::Marker text_marker;
  text_marker.header = header;
  text_marker.header.frame_id = player_frame_id_;
  text_marker.ns = mob_text_namespace_;
  text_marker.id = static_cast<int>(entity.id);
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;

  text_marker.pose = entity.pose;

  auto transformed_position = this->transformToPlayerFrame(
    player_orientation,
    entity.pose.position.x - player_x,
    entity.pose.position.y - player_y,
    entity.pose.position.z - player_z
  );

  text_marker.pose.position.x = transformed_position.getX();
  text_marker.pose.position.y = transformed_position.getY();
  text_marker.pose.position.z = transformed_position.getZ();
  text_marker.pose.position.z += entity.hit_box.z / 2.0 + 0.5;

  text_marker.scale.z = 0.3;

  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  std::stringstream ss;
  ss << entity.name << " ("
     << static_cast<int>(entity.health) << "/"
     << static_cast<int>(entity.max_health) << ")";
  text_marker.text = ss.str();

  text_marker.lifetime = rclcpp::Duration(1, 0);

  marker_array.markers.push_back(text_marker);
}


tf2::Vector3 MobMarker::transformToPlayerFrame(
  const geometry_msgs::msg::Quaternion & player_orientation,
  const float x, const float y, const float z)
{
  tf2::Quaternion tf_quat;
  tf_quat.setValue(
    player_orientation.x,
    player_orientation.y,
    player_orientation.z,
    player_orientation.w
  );

  tf2::Matrix3x3 rot_matrix(tf_quat.inverse());
  tf2::Vector3 point(x, y, z);
  tf2::Vector3 transformed_point = rot_matrix * point;

  return transformed_point;
}

}  // namespace minecraft_utils_visualization


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(minecraft_utils_visualization::MobMarker)

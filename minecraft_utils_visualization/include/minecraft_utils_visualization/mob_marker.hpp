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

#include <cmath>
#include <functional>
#include <sstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <minecraft_msgs/msg/living_entity.hpp>
#include <minecraft_msgs/msg/living_entity_array.hpp>
#include <minecraft_msgs/msg/mob_category.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>


namespace minecraft_utils_visualization
{
class MobMarker : public rclcpp::Node
{
public:
    explicit MobMarker(rclcpp::NodeOptions);
    ~MobMarker() = default;

private:
    void mobCallback(const minecraft_msgs::msg::LivingEntityArray::SharedPtr);
  
    void setMarkerColor(const uint8_t, visualization_msgs::msg::Marker &);
  
    void addNameHealthMarker(
        const minecraft_msgs::msg::LivingEntity &,
        const std_msgs::msg::Header &,
        const float, const float, const float,
        const geometry_msgs::msg::Quaternion &,
        visualization_msgs::msg::MarkerArray &);
    
    tf2::Vector3 transformToPlayerFrame(
        const geometry_msgs::msg::Quaternion& player_orientation,
        const float, const float, const float);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<minecraft_msgs::msg::LivingEntityArray>::SharedPtr mob_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string world_frame_id_;
    std::string player_frame_id_;
    std::string mob_namespace_;
    std::string mob_text_namespace_;
};

}  // namespace minecraft_utils_visualization

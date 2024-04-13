#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// -- ros messages --
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

// -- tf2 stuff --
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;
using namespace std;

class MapperNode : public rclcpp::Node
{
public:
  MapperNode()
      : Node("mapper_node")
  {
    // -- create subscribers --
    depthImageSub = this->create_subscription<sensor_msgs::msg::Image>(
        depthTopic, 10, std::bind(&MapperNode::depthImageCb, this, _1));

    // -- create publishers --
    mapPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mapTopic, 10);
    candidateMapPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(candidateMapTopic, 10);

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map.header.frame_id = "map";
    map.header.stamp = this->get_clock()->now();
    map.info.resolution = 0.025;
    map.info.width = 280;
    map.info.height = 150;
    map.info.origin.position.x = -(float)map.info.width * map.info.resolution / 2.0f + 1.0f;
    map.info.origin.position.y = -(float)map.info.height * map.info.resolution / 2.0f;
    map.info.origin.position.z = 0;
    map.data = std::vector<int8_t>(map.info.width * map.info.height, 0);

    candidateMap = map;

    rclcpp::Rate rate(0.2);
    rate.sleep();

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MapperNode::publishMap, this));
  }

private:
  void publishMap()
  {
    RCLCPP_INFO(this->get_logger(), "Publishing the map.");
    map.header.stamp = this->get_clock()->now();
    candidateMap.header.stamp = this->get_clock()->now();
    mapPub->publish(map);
    candidateMapPub->publish(candidateMap);
  }

  void depthImageCb(const sensor_msgs::msg::Image::SharedPtr image)
  {
    RCLCPP_INFO(this->get_logger(), "Received a depth image.");

    // -- parse raw data into depth array --
    uint8_t *data = image->data.data();
    uint16_t *depth = (uint16_t *)data; // depth container, values in [mm], number of values = image->height * image->width, row major order
    uint32_t counter = 0;
    for (uint32_t row = 0; row < image->height; row++)
    {
      for (uint32_t col = 0; col < image->step; col += 2) // we advance by 2 because two uint8_t make a uint16_t
      {
        // -- NOTE: we know the data is little-endian --
        uint16_t thisDepth = ((uint16_t)data[row * image->step + col]) | ((uint16_t)data[row * image->step + col + 1] << 8);
        depth[counter] = thisDepth;
        counter++;
      }
    }

    // -- filter data by the following conditions --
    // -- (1) invalid distance (i.e. too far or too close) --
    // -- (2) above the ground, and within some height --
    std::vector<geometry_msgs::msg::Point> validPoints;
    for (uint32_t row = 0; row < image->height; row++)
    {
      for (uint32_t col = 0; col < image->width; col++)
      {
        uint32_t index = row * image->width + col;
        float XYZCamera[3] = {0};
        computeCoordinates(col, row, depth[index], XYZCamera);

        bool isValid = false;
        if (depth[index] < VALID_DEPTH_THRESHOLDS[0] || depth[index] > VALID_DEPTH_THRESHOLDS[1])
        {
          isValid = false;
        }
        else if (!(XYZCamera[1] > VALID_HEIGHT_IN_CAMERA_THRESHOLDS[0] && XYZCamera[1] < VALID_HEIGHT_IN_CAMERA_THRESHOLDS[1]))
        {
          isValid = false;
        }
        else
        {
          isValid = true;
        }

        if (isValid)
        {
          geometry_msgs::msg::Point point;
          point.x = XYZCamera[0];
          point.y = XYZCamera[1];
          point.z = XYZCamera[2];
          validPoints.push_back(point);
        }
      }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Number of valid points: " << validPoints.size());

    // -- transform the 3D points from local frame into the map frame --
    geometry_msgs::msg::TransformStamped transform;
    geometry_msgs::msg::TransformStamped rob_loc;
    try
    {
      transform = tf_buffer_->lookupTransform(
          "map", image->header.frame_id, image->header.stamp);

      rob_loc = tf_buffer_->lookupTransform(
          "map", "base_link", image->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(
          this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }
    // RCLCPP_INFO(
    //     this->get_logger(), "Transform success");
    float rob_x = rob_loc.transform.translation.x;
    float rob_y = rob_loc.transform.translation.y;

    std::vector<geometry_msgs::msg::Point> transformedPoints;
    uint16_t printCnt = 0;
    for (auto point : validPoints)
    {
      geometry_msgs::msg::Point transformedPoint;
      tf2::doTransform(point, transformedPoint, transform);
      transformedPoints.push_back(transformedPoint);
      printCnt++;
      if (printCnt < 30)
      {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Transformed point: " << transformedPoint.x << ", " << transformedPoint.y << ", " << transformedPoint.z);
      }
    }

    // -- update the candidate map with the new points --
    std::vector<uint8_t> updateMask(map.info.width * map.info.height, 0);
    for (auto point : transformedPoints)
    {
      // -- convert the 3D point into the map frame --
      int32_t xMap = (point.x / 1000.0f - map.info.origin.position.x) / map.info.resolution;
      int32_t yMap = (point.y / 1000.0f - map.info.origin.position.y) / map.info.resolution;
      // RCLCPP_INFO_STREAM(this->get_logger(), "Point in the map frame: " << xMap << ", " << yMap);
      // -- update the candidate map --
      if (xMap >= 0 && xMap < map.info.width && yMap >= 0 && yMap < map.info.height)
      {
        if (candidateMap.data[yMap * map.info.width + xMap] < 100)
        {
          // RCLCPP_INFO_STREAM(this->get_logger(), "Updating the candidate map at: " << xMap << ", " << yMap << " with value: " << (int)candidateMap.data[yMap * map.info.width + xMap] + 1);
          candidateMap.data[yMap * map.info.width + xMap] += 1;
          updateMask[yMap * map.info.width + xMap] = 1;
        }
      }
    }

    // -- update the candidate map considering false detection --
    for (uint32_t row = 0; row < map.info.height; row++)
    {
      for (uint32_t col = 0; col < map.info.width; col++)
      {
        if (updateMask[row * map.info.width + col] == 0)
        {
          float x = map.info.origin.position.x + col * map.info.resolution;
          float y = map.info.origin.position.y + row * map.info.resolution;
          float distance = sqrt(pow(x - rob_x, 2) + pow(y - rob_y, 2));
          if (candidateMap.data[row * map.info.width + col] > 2)
            candidateMap.data[row * map.info.width + col] -= 2;
          else
            candidateMap.data[row * map.info.width + col] = 0;
        }
        else if ((candidateMap.data[row * map.info.width + col] + 1) < 100)
        {
          candidateMap.data[row * map.info.width + col] += 1;
        }
      }
    }

    // -- update the map with the candidate map --
    for (uint32_t row = 0; row < map.info.height; row++)
    {
      for (uint32_t col = 0; col < map.info.width; col++)
      {
        if (candidateMap.data[row * map.info.width + col] > CONCLUDE_EXISTENCE_THRESHOLD)
        {
          map.data[row * map.info.width + col] = 100;
        }
        else if (candidateMap.data[row * map.info.width + col] == 0)
        {
          map.data[row * map.info.width + col] = 0;
        }
      }
    }

    // -- publish the map and the candidate map--
  }

  void computeCoordinates(uint16_t xImage, uint16_t yImage, uint16_t zCamera, float outXYZ[3])
  {
    // -- compute the 3D coordinates of a point in the camera frame --
    float xCamera = (xImage - CAMERA_MATRIX[2]) * zCamera / CAMERA_MATRIX[0];
    float yCamera = (yImage - CAMERA_MATRIX[5]) * zCamera / CAMERA_MATRIX[4];

    outXYZ[0] = xCamera;
    outXYZ[1] = yCamera;
    outXYZ[2] = zCamera;
  }

  // -- hard-coded camera matrix for the depth camera --
  const float CAMERA_MATRIX[9] = {392.4931335449219, 0, 320.5128479003906, 0, 392.4931335449219, 236.08059692382812, 0, 0, 1};
  const float VALID_DEPTH_THRESHOLDS[2] = {50, 3500};            // in [mm]
  const float VALID_HEIGHT_IN_CAMERA_THRESHOLDS[2] = {-40, 58.5}; // in [mm]
  uint32_t CONCLUDE_EXISTENCE_THRESHOLD = 35;

  nav_msgs::msg::OccupancyGrid map;
  nav_msgs::msg::OccupancyGrid candidateMap;

  string depthTopic = "/camera/depth/image_rect_raw";
  string mapTopic = "/map";
  string candidateMapTopic = "/candidate_map";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr candidateMapPub;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapperNode>());
  rclcpp::shutdown();

  printf("hello world mapper package\n");
  return 0;
}

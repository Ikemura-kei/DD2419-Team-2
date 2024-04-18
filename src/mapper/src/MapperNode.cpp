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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// -- tf2 stuff --
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;
using namespace std;
using namespace chrono;

union FloatUint8
{
  float floatRep;
  uint32_t uint32Rep;
  uint8_t uint8Rep[4];
} floatUint8;

class MapperNode : public rclcpp::Node
{
public:
  MapperNode()
      : Node("mapper_node")
  {
    // -- create subscribers --
    // depthImageSub = this->create_subscription<sensor_msgs::msg::Image>(
    //     depthTopic, 10, std::bind(&MapperNode::depthImageCb, this, _1));
    scanSub = this->create_subscription<sensor_msgs::msg::LaserScan>(scanTopic, 10, std::bind(&MapperNode::scanCb, this, _1));
    pntCldSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(pntCldTopic, 10, std::bind(&MapperNode::pntCldCb, this, _1));

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

    // rclcpp::Rate rate(0.2);
    // rate.sleep();

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MapperNode::publishMap, this));
  }

private:
  void publishMap()
  {
    // RCLCPP_INFO(this->get_logger(), "Publishing the map.");
    map.header.stamp = this->get_clock()->now();
    candidateMap.header.stamp = this->get_clock()->now();
    mapPub->publish(map);
    candidateMapPub->publish(candidateMap);
  }

  void pntCldCb(const sensor_msgs::msg::PointCloud2::SharedPtr pntCld)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Received point cloud data.");
    int SKIP_STEP = 7;

    auto parsePntStartT = high_resolution_clock::now();
    // -- parse points --
    std::vector<geometry_msgs::msg::Point> points;
    int newStep = pntCld->point_step * SKIP_STEP;
    for (int i = 0; i < pntCld->row_step; i += newStep)
    {
      geometry_msgs::msg::Point point;

      for (int j = 0; j < 4; j++)
        floatUint8.uint8Rep[j] = pntCld->data[i + j + 4]; // somehow floats are little-endian
      point.y = floatUint8.floatRep;

      if (!((point.y * 1000) > VALID_HEIGHT_IN_CAMERA_THRESHOLDS[0] && (point.y * 1000) < VALID_HEIGHT_IN_CAMERA_THRESHOLDS[1]))
      {
        continue;
      }

      for (int j = 0; j < 4; j++)
        floatUint8.uint8Rep[j] = pntCld->data[i + j + 8]; // somehow floats are little-endian
      point.z = floatUint8.floatRep;

      if ((point.z * 1000) < VALID_DEPTH_THRESHOLDS[0] || (point.z * 1000) > VALID_DEPTH_THRESHOLDS[1])
      {
        continue;
      }

      for (int j = 0; j < 4; j++)
        floatUint8.uint8Rep[j] = pntCld->data[i + j]; // somehow floats are little-endian
      point.x = floatUint8.floatRep;

      points.push_back(point);
    }

    // -- transform points into the map frame --
    geometry_msgs::msg::TransformStamped base2MapTransform;
    try
    {
      base2MapTransform = this->tf_buffer_->lookupTransform("map", pntCld->header.frame_id, pntCld->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
    }

    std::vector<geometry_msgs::msg::Point> mapPoints;
    for (auto point : points)
    {
      geometry_msgs::msg::Point outPoint;
      tf2::doTransform(point, outPoint, base2MapTransform);
      mapPoints.push_back(outPoint);
    }
    auto parsePntEndT = high_resolution_clock::now();

    auto updateMapStartT = high_resolution_clock::now();
    // -- update map accordingly --
    updateMap(mapPoints);
    auto updateMapEndT = high_resolution_clock::now();

    auto pntProcT = duration_cast<microseconds>(parsePntEndT - parsePntStartT);
    auto pntMapUpdateT = duration_cast<microseconds>(updateMapEndT - updateMapStartT);
    RCLCPP_INFO_STREAM(this->get_logger(), "Point cloud processing takes: " << pntProcT.count() / 1e6 << " seconds, and map update takes: " << pntMapUpdateT.count() / 1e6 << " seconds");
  }

  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    auto scanProcStartT = high_resolution_clock::now();
    // -- get points from range-bearing data --
    std::vector<geometry_msgs::msg::Point> points;
    float totalBearing = scan->angle_min;
    for (int i = 0; i < scan->ranges.size(); i++)
    {
      float range = scan->ranges[i];
      if (isnan(range) || isinf(range) || range < 0.45)
      {
        totalBearing += scan->angle_increment;
        continue;
      }

      geometry_msgs::msg::Point point;
      point.x = range * cos(totalBearing);
      point.y = range * sin(totalBearing);
      point.z = 0;
      points.push_back(point);

      totalBearing += scan->angle_increment;

      // RCLCPP_INFO_STREAM(this->get_logger(), "The point computed is: " << point.x << ", " << point.y);
    }

    // -- transform the points to the map frame --
    geometry_msgs::msg::TransformStamped base2MapTransform;
    try
    {
      base2MapTransform = this->tf_buffer_->lookupTransform("map", scan->header.frame_id, scan->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
    }

    std::vector<geometry_msgs::msg::Point> mapPoints;
    for (auto point : points)
    {
      geometry_msgs::msg::Point outPoint;
      tf2::doTransform(point, outPoint, base2MapTransform);
      mapPoints.push_back(outPoint);
    }
    auto scanProcEndT = high_resolution_clock::now();

    auto scanMapUpdateStartT = high_resolution_clock::now();
    // -- add to candidate map and update the map accordingly --
    // updateMap(mapPoints, true);
    auto scanMapUpdateEndT = high_resolution_clock::now();

    auto scanProcT = duration_cast<microseconds>(scanMapUpdateEndT - scanMapUpdateStartT);
    auto scanMapUpdateT = duration_cast<microseconds>(scanProcEndT - scanProcStartT);
    RCLCPP_INFO_STREAM(this->get_logger(), "Scan processing takes: " << scanProcT.count() / 1e6 << " seconds, and map update takes: " << scanMapUpdateT.count() / 1e6 << " seconds");
  }

  void depthImageCb(const sensor_msgs::msg::Image::SharedPtr image)
  {
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

    float rob_x = rob_loc.transform.translation.x;
    float rob_y = rob_loc.transform.translation.y;

    std::vector<geometry_msgs::msg::Point> transformedPoints;
    for (auto point : validPoints)
    {
      geometry_msgs::msg::Point transformedPoint;
      tf2::doTransform(point, transformedPoint, transform);
      // -- mm to m --
      transformedPoint.x = transformedPoint.x / 1000.0f;
      transformedPoint.y = transformedPoint.y / 1000.0f;
      transformedPoints.push_back(transformedPoint);
    }

    updateMap(transformedPoints);
  }

  void updateMap(std::vector<geometry_msgs::msg::Point> &points, bool isScan = false)
  {
    // -- update the candidate map with the new points --
    std::vector<uint8_t> updateMask(map.info.width * map.info.height, 0);
    for (auto point : points)
    {
      if (point.y <= 0.08 && point.y >= -0.08)
      {
        continue;
      }

      // -- convert the 3D point into the map frame --
      int32_t xMap = (point.x - map.info.origin.position.x) / map.info.resolution;
      int32_t yMap = (point.y - map.info.origin.position.y) / map.info.resolution;

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
          if (!isScan)
          {
            if (candidateMap.data[row * map.info.width + col] > 2)
              candidateMap.data[row * map.info.width + col] -= 2;
            else
              candidateMap.data[row * map.info.width + col] = 0;
          }
        }
        else if ((candidateMap.data[row * map.info.width + col] + 5) < 100)
        {
          if (isScan)
            candidateMap.data[row * map.info.width + col] += 0;
          else
            candidateMap.data[row * map.info.width + col] += 3;
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
  const float VALID_DEPTH_THRESHOLDS[2] = {35, 3570};             // in [mm]
  const float VALID_HEIGHT_IN_CAMERA_THRESHOLDS[2] = {-40, 58.5}; // in [mm]
  uint32_t CONCLUDE_EXISTENCE_THRESHOLD = 35;

  nav_msgs::msg::OccupancyGrid map;
  nav_msgs::msg::OccupancyGrid candidateMap;

  string depthTopic = "/camera/depth/image_rect_raw";
  string scanTopic = "/scan";
  string mapTopic = "/map";
  string pntCldTopic = "/camera/depth/color/points";
  string candidateMapTopic = "/candidate_map";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pntCldSub;
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

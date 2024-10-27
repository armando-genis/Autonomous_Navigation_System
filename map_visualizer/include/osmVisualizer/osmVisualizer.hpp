
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// lanelet libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_projection/LocalCartesian.h>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>

#include <lanelet2_core/geometry/Point.h>

#include <lanelet2_core/primitives/Lanelet.h>

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <boost/optional/optional_io.hpp>

// we want assert statements to work in release mode
#undef NDEBUG

using namespace std::chrono_literals;
using namespace lanelet;

class OsmVisualizer : public rclcpp::Node
{
public:
  OsmVisualizer();

private:
  void timer_callback();
  bool readParameters();
  void writeToFile(const std_msgs::msg::Float64MultiArray &multi_array);
  void fill_marker(lanelet::LaneletMapPtr &t_map);
  void fill_array(lanelet::LaneletMapPtr &t_map);
  void fill_array_with_left_right(lanelet::LaneletMapPtr &t_map);
  double getDistance(const lanelet::ConstLanelet &ll, size_t i);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_publisher_;
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygon_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_publisher_;

  std_msgs::msg::Float64MultiArray m_array;
  visualization_msgs::msg::MarkerArray m_marker_array;
  polygon_msgs::msg::Polygon2DCollection crosswalk_polygons;

  // colors for the terminal
  std::string green = "\033[1;32m";
  std::string red = "\033[1;31m";
  std::string blue = "\033[1;34m";
  std::string yellow = "\033[1;33m";
  std::string purple = "\033[1;35m";
  std::string reset = "\033[0m";

  // params
  std::string map_path_;
  bool enable_inc_path_points_;
  double interval_;
};
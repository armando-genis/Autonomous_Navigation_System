#include "../include/osmVisualizer/osmVisualizer.hpp"

OsmVisualizer::OsmVisualizer() : Node("OsmVisualizer")
{
  this->declare_parameter("map_path", "/home/atakan/Downloads/Town10.osm");
  this->declare_parameter("enable_inc_path_points", true);
  this->declare_parameter("interval", 2.0);
  if (!readParameters())
    rclcpp::shutdown();

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map", 10);
  array_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/array", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&OsmVisualizer::timer_callback, this));

  polygon_publisher_ = this->create_publisher<polygon_msgs::msg::Polygon2DCollection>("/crosswalk_polygons", 10);

  // Load the map
  // lanelet::projection::UtmProjector projector(lanelet::Origin({49.0, 8.0}));

  // // Load the map using the UTM projector
  // lanelet::LaneletMapPtr map = lanelet::load(map_path_, projector);

  lanelet::Origin origin({49.0, 8.0});
  lanelet::projection::LocalCartesianProjector projector(origin);
  lanelet::LaneletMapPtr map = lanelet::load(map_path_, projector);

  for (auto &point : map->pointLayer)
  {
    point.x() = point.attribute("local_x").asDouble().value();
    point.y() = point.attribute("local_y").asDouble().value();
  }

  fill_marker(map);
  fill_array_with_left_right(map);
  // writeToFile(m_array);
}

bool OsmVisualizer::readParameters()
{
  if (!this->get_parameter("map_path", map_path_))
  {
    std::cout << "Failed to read parameter 'map_path' " << std::endl;
    return false;
  }
  if (!this->get_parameter("enable_inc_path_points", enable_inc_path_points_))
  {
    std::cout << "Failed to read parameter 'interval' to increase the path points" << std::endl;
    return false;
  }
  if (!this->get_parameter("interval", interval_))
  {
    std::cout << "Failed to read parameter 'interval' to increase the path points" << std::endl;
    return false;
  }
  return true;
}

void OsmVisualizer::timer_callback()
{
  publisher_->publish(m_marker_array);
  array_publisher_->publish(m_array);
  if (!crosswalk_polygons.polygons.empty())
  {
    polygon_publisher_->publish(crosswalk_polygons);
    std::cout << "Published crosswalk polygons" << std::endl;
  }
}

void OsmVisualizer::fill_array(lanelet::LaneletMapPtr &t_map)
{
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[0].label = "rows";
  m_array.layout.dim[0].size = 100000;
  m_array.layout.dim[0].stride = 100000 * 2;
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[1].label = "cols";
  m_array.layout.dim[1].size = 2;
  m_array.layout.dim[1].stride = 2;

  for (const auto &ll : t_map->laneletLayer)
  {
    for (size_t i = 0; i < ll.centerline2d().size() - 1; i++)
    {
      if (getDistance(ll, i) > 2 && enable_inc_path_points_)
      {
        double dist = getDistance(ll, i);
        double interval = 1;
        int num_points = dist / interval;

        for (int k = 0; k < num_points; k++)
        {
          m_array.data.push_back(((ll.centerline2d()[i + 1].x() - ll.centerline2d()[i].x()) / num_points) * k + ll.centerline2d()[i].x());
          m_array.data.push_back(((ll.centerline2d()[i + 1].y() - ll.centerline2d()[i].y()) / num_points) * k + ll.centerline2d()[i].y());
        }
      }
      else
      {
        m_array.data.push_back(ll.centerline2d()[i].x());
        m_array.data.push_back(ll.centerline2d()[i].y());
      }
    }
  }
}

void OsmVisualizer::writeToFile(const std_msgs::msg::Float64MultiArray &multi_array)
{
  std::ofstream file("data.txt");
  if (file.is_open())
  {
    for (size_t i = 0; i < multi_array.data.size(); ++i)
    {
      file << multi_array.data[i] << ",";
      if ((i + 1) % (multi_array.layout.dim[0].size) == 0)
        file << "\n";
      if ((i + 1) % (multi_array.layout.dim[1].size) == 0)
        file << "\n";
    }
    file.close();
  }
}

void OsmVisualizer::fill_array_with_left_right(lanelet::LaneletMapPtr &t_map)
{
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[0].label = "rows";
  m_array.layout.dim[0].size = t_map->laneletLayer.size();
  m_array.layout.dim[0].stride = t_map->laneletLayer.size() * 4;
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[1].label = "cols";
  m_array.layout.dim[1].size = 4;
  m_array.layout.dim[1].stride = 4;

  for (const auto &ll : t_map->laneletLayer)
  {
    std::vector<lanelet::ConstLineString3d> bounds;
    bounds.push_back(ll.leftBound());
    bounds.push_back(ll.rightBound());

    size_t size = (bounds[0].size() < bounds[1].size()) ? bounds[0].size() : bounds[1].size();
    for (size_t i = 0; i < size; i++)
    {
      m_array.data.push_back(bounds[0][i].x());
      m_array.data.push_back(bounds[0][i].y());
      m_array.data.push_back(bounds[1][i].x());
      m_array.data.push_back(bounds[1][i].y());
    }
  }
}

double OsmVisualizer::getDistance(const lanelet::ConstLanelet &ll, size_t i)
{
  return std::sqrt(std::pow(ll.centerline2d()[i].x() - ll.centerline2d()[i + 1].x(), 2) + std::pow(ll.centerline2d()[i].y() - ll.centerline2d()[i + 1].y(), 2));
}

void OsmVisualizer::fill_marker(lanelet::LaneletMapPtr &t_map)
{

  size_t i = 0;
  int crosswalk_count = 0;             // Counter for crosswalk subtype
  crosswalk_polygons.polygons.clear(); // Clear the crosswalk polygons

  crosswalk_polygons.header.stamp = rclcpp::Clock{}.now();
  crosswalk_polygons.header.frame_id = "map";

  std_msgs::msg::ColorRGBA color;
  color.r = 0.6; // Red color
  color.g = 0.6; // No green
  color.b = 0.6; // No blue
  color.a = 0.5; // Full opacity

  crosswalk_polygons.colors.clear();
  crosswalk_polygons.colors.push_back(color);

  double base_polygon_z_offset = 0.0;   // Z-offset for the base crosswalk polygon
  double stripe_polygon_z_offset = 0.1; // Z-offset for the zebra crossing stripes

  // Iterate over the lanelets in the map
  for (const auto &ll : t_map->laneletLayer)
  {

    std::vector<lanelet::ConstLineString3d> bounds;
    bounds.push_back(ll.leftBound());
    bounds.push_back(ll.rightBound());

    // Check if the lanelet has the subtype 'crosswalk'
    if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
        ll.attribute(lanelet::AttributeName::Subtype).value() == lanelet::AttributeValueString::Crosswalk)
    {
      crosswalk_count++; // Increment the crosswalk counter
      // Create a Polygon2D for the crosswalk
      polygon_msgs::msg::Polygon2D polygon;
      polygon.z_offset = 3.0;

      // For the left bound
      for (const auto &point : ll.leftBound())
      {
        polygon_msgs::msg::Point2D p;
        p.x = point.x(); // Convert from lanelet point to polygon_msgs Point2D
        p.y = point.y();
        polygon.points.push_back(p); // Add to the polygon's points
      }

      // For the right bound
      const auto &right_bound = ll.rightBound();
      for (int i = right_bound.size() - 1; i >= 0; --i)
      {
        polygon_msgs::msg::Point2D p;
        p.x = right_bound[i].x(); // Convert from lanelet point to polygon_msgs Point2D
        p.y = right_bound[i].y();
        polygon.points.push_back(p); // Add to the polygon's points
      }

      // close the loop by adding the first point to the end
      polygon.points.push_back(polygon.points[0]);

      // crosswalk_polygons.polygons.push_back(polygon);

      double max_z = std::numeric_limits<double>::lowest(); // Initialize to the lowest possible value
      for (const auto &point : ll.leftBound())
      {
        // std::cout << "Point z: " << point.z() << std::endl; // Print each z value

        if (point.z() > max_z)
        {
          max_z = point.z();
        }
      }

      std::cout << "----->max_z set polygon: " << max_z << std::endl;

      // Create zebra crossing stripes
      int num_stripes = 9;         // Number of stripes
      double stripe_spacing = 0.5; // Space between stripes
      double initial_gap = 0.1;    // Initial separation (gap) before the first stripe

      // For each stripe
      for (int i = 0; i < num_stripes; i++)
      {
        double t = i / static_cast<double>(num_stripes); // Parameter for interpolation

        // Interpolate between left and right bounds to create the stripe
        polygon_msgs::msg::Point2D left_start, right_start, left_end, right_end;

        // Calculate the direction of the edge (between the first and last points of the left bound)
        double dx_left = ll.leftBound().back().x() - ll.leftBound()[0].x();
        double dy_left = ll.leftBound().back().y() - ll.leftBound()[0].y();

        // Normalize the direction vector for the left bound
        double length_left = std::sqrt(dx_left * dx_left + dy_left * dy_left);
        dx_left /= length_left;
        dy_left /= length_left;

        // Interpolate along the left bound
        left_start.x = (1 - t) * ll.leftBound()[0].x() + t * ll.leftBound().back().x() + initial_gap * dx_left;
        left_start.y = (1 - t) * ll.leftBound()[0].y() + t * ll.leftBound().back().y() + initial_gap * dy_left;

        // Same for the right bound
        double dx_right = ll.rightBound().back().x() - ll.rightBound()[0].x();
        double dy_right = ll.rightBound().back().y() - ll.rightBound()[0].y();

        double length_right = std::sqrt(dx_right * dx_right + dy_right * dy_right);
        dx_right /= length_right;
        dy_right /= length_right;

        right_start.x = (1 - t) * ll.rightBound()[0].x() + t * ll.rightBound().back().x() + initial_gap * dx_right;
        right_start.y = (1 - t) * ll.rightBound()[0].y() + t * ll.rightBound().back().y() + initial_gap * dy_right;

        // Create the stripe end points based on the stripe width
        left_end.x = left_start.x + stripe_spacing * dx_left;
        left_end.y = left_start.y + stripe_spacing * dy_left;

        right_end.x = right_start.x + stripe_spacing * dx_right;
        right_end.y = right_start.y + stripe_spacing * dy_right;

        // Create a stripe polygon
        polygon_msgs::msg::Polygon2D stripe_polygon;
        stripe_polygon.z_offset = max_z; // set the ele value of the polygon to the min z value of the lanelet

        stripe_polygon.points.push_back(left_start);
        stripe_polygon.points.push_back(right_start);
        stripe_polygon.points.push_back(right_end);
        stripe_polygon.points.push_back(left_end);
        stripe_polygon.points.push_back(left_start); // Close the loop

        // Alternate between white and gray stripes
        std_msgs::msg::ColorRGBA stripe_color;
        stripe_color.r = 1.0;
        stripe_color.g = 1.0;
        stripe_color.b = 1.0;
        stripe_color.a = 0.8; // Slightly opaque

        // Add the stripe polygon to the crosswalk polygons
        crosswalk_polygons.polygons.push_back(stripe_polygon);

        // Add the stripe color to the colors list
        crosswalk_polygons.colors.push_back(stripe_color);
      }

      // for to cout the z_offset of the polygon
      for (size_t i = 0; i < crosswalk_polygons.polygons.size(); i++)
      {
        std::cout << "----->z_offset set polygon: " << crosswalk_polygons.polygons[i].z_offset << std::endl;
      }
    }
    else
    {
      // For each bound (left and right), create a marker
      for (const auto &bound : bounds)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock{}.now();
        marker.ns = "lanelet";
        marker.id = i++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        // Add the points to the marker
        for (const auto &point : bound)
        {
          geometry_msgs::msg::Point p;
          p.x = point.x();
          p.y = point.y();
          p.z = point.z();
          marker.points.push_back(p);
        }

        // Add the marker to the array
        m_marker_array.markers.push_back(marker);
      }
    }
  }
  std::cout << "----> Number of crosswalk lanelets: " << crosswalk_count << std::endl;
}

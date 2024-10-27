std::vector<lanelet::ConstLineString3d> bounds;
bounds.push_back(ll.leftBound());
bounds.push_back(ll.rightBound());

// Check if the lanelet has the subtype 'crosswalk'
if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
    ll.attribute(lanelet::AttributeName::Subtype).value() == lanelet::AttributeValueString::Crosswalk)
{
    crosswalk_count++; // Increment the crosswalk counter
    polygon_msgs::msg::Polygon2D polygon;
    polygon.z_offset = 3.0;

    int num_stripes = 3; // Number of stripes in the zebra crossing

    double max_z = std::numeric_limits<double>::lowest();
    for (const auto &point : ll.leftBound())
    {
        if (point.z() > max_z)
        {
            max_z = point.z();
        }
    }

    std::cout << "----->max_z set polygon: " << max_z << std::endl;

    // Calculate the total length of the left and right bounds
    double left_bound_length = 0.0;
    for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
    {
        left_bound_length += std::sqrt(
            std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
            std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2));
    }

    // Determine stripe width based on total length and number of stripes
    double stripe_width = left_bound_length / num_stripes;

    // Start creating stripes by interpolation along the left and right bounds
    double accumulated_length = 0.0;
    size_t left_index = 0, right_index = 0;
    while (left_index < ll.leftBound().size() - 1 && right_index < ll.rightBound().size() - 1)
    {
        // Calculate next segment length on left bound
        double left_segment_length = std::sqrt(
            std::pow(ll.leftBound()[left_index + 1].x() - ll.leftBound()[left_index].x(), 2) +
            std::pow(ll.leftBound()[left_index + 1].y() - ll.leftBound()[left_index].y(), 2));

        // Calculate next segment length on right bound
        double right_segment_length = std::sqrt(
            std::pow(ll.rightBound()[right_index + 1].x() - ll.rightBound()[right_index].x(), 2) +
            std::pow(ll.rightBound()[right_index + 1].y() - ll.rightBound()[right_index].y(), 2));

        // Check if we have accumulated enough length for a stripe
        if (accumulated_length + left_segment_length >= stripe_width)
        {
            // Calculate interpolation factor for left and right bounds
            double remaining_length = stripe_width - accumulated_length;
            double left_interp_factor = remaining_length / left_segment_length;
            double right_interp_factor = remaining_length / right_segment_length;

            // Calculate stripe corners on the left and right bounds
            polygon_msgs::msg::Point2D left_start, right_start, left_end, right_end;

            // Start points (current positions)
            left_start.x = ll.leftBound()[left_index].x();
            left_start.y = ll.leftBound()[left_index].y();

            right_start.x = ll.rightBound()[right_index].x();
            right_start.y = ll.rightBound()[right_index].y();

            // End points (interpolated positions)
            left_end.x = ll.leftBound()[left_index].x() + left_interp_factor *
                                                              (ll.leftBound()[left_index + 1].x() - ll.leftBound()[left_index].x());
            left_end.y = ll.leftBound()[left_index].y() + left_interp_factor *
                                                              (ll.leftBound()[left_index + 1].y() - ll.leftBound()[left_index].y());

            right_end.x = ll.rightBound()[right_index].x() + right_interp_factor *
                                                                 (ll.rightBound()[right_index + 1].x() - ll.rightBound()[right_index].x());
            right_end.y = ll.rightBound()[right_index].y() + right_interp_factor *
                                                                 (ll.rightBound()[right_index + 1].y() - ll.rightBound()[right_index].y());

            // Form the stripe polygon
            polygon_msgs::msg::Polygon2D stripe_polygon;
            stripe_polygon.z_offset = max_z;

            // Add points in a consistent order to form a rectangle
            stripe_polygon.points.push_back(left_start);
            stripe_polygon.points.push_back(right_start);
            stripe_polygon.points.push_back(right_end);
            stripe_polygon.points.push_back(left_end);
            stripe_polygon.points.push_back(left_start); // Close the loop

            // Set color for the stripe
            std_msgs::msg::ColorRGBA stripe_color;
            stripe_color.r = 1.0;
            stripe_color.g = 1.0;
            stripe_color.b = 1.0;
            stripe_color.a = 0.8;

            // Add the stripe to the crosswalk polygon collection
            crosswalk_polygons.polygons.push_back(stripe_polygon);
            crosswalk_polygons.colors.push_back(stripe_color);

            // Reset accumulated length
            accumulated_length = 0.0;
        }
        else
        {
            // Accumulate the length and move to the next segment
            accumulated_length += left_segment_length;
            ++left_index;
            ++right_index;
        }
    }
}

// =================================================================================================

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
            crosswalk_count++;   // Increment the crosswalk counter
            int num_stripes = 5; // Number of stripes in the zebra crossing

            double max_z = std::numeric_limits<double>::lowest();
            for (const auto &point : ll.leftBound())
            {
                if (point.z() > max_z)
                {
                    max_z = point.z();
                }
            }

            std::cout << "----->max_z set polygon: " << max_z << std::endl;

            // Calculate the total length of the left and right bounds
            double left_bound_length = 0.0;
            for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
            {
                left_bound_length += std::sqrt(
                    std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
                    std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2) +
                    std::pow(ll.leftBound()[i + 1].z() - ll.leftBound()[i].z(), 2));
            }

            // double lenght_theshold = 0.5;

            std::cout << "----->left_bound_length: " << left_bound_length << std::endl;

            double stripe_length = left_bound_length / (2 * num_stripes);

            // Generate zebra stripes
            for (int stripe_idx = 0; stripe_idx < num_stripes; ++stripe_idx)
            {
                polygon_msgs::msg::Polygon2D stripe_polygon;
                stripe_polygon.z_offset = 3.0;

                // Calculate the start and end points for the current stripe on the left and right bounds
                double start_dist = stripe_idx * 2 * stripe_length;
                double end_dist = start_dist + stripe_length;

                // Add points for the stripe from the left bound
                double accumulated_length = 0.0;
                polygon_msgs::msg::Point2D start_left, end_left;
                bool start_left_set = false, end_left_set = false;

                for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
                {
                    double segment_length = std::sqrt(
                        std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
                        std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2));

                    if (accumulated_length + segment_length > start_dist && !start_left_set)
                    {
                        double ratio = (start_dist - accumulated_length) / segment_length;
                        start_left.x = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
                        start_left.y = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
                        start_left_set = true;
                    }

                    if (accumulated_length + segment_length > end_dist && !end_left_set)
                    {
                        double ratio = (end_dist - accumulated_length) / segment_length;
                        end_left.x = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
                        end_left.y = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
                        end_left_set = true;
                        break;
                    }

                    accumulated_length += segment_length;
                }

                if (start_left_set && end_left_set)
                {
                    stripe_polygon.points.push_back(start_left);
                    stripe_polygon.points.push_back(end_left);
                }

                // Add points for the stripe from the right bound
                accumulated_length = 0.0;
                polygon_msgs::msg::Point2D start_right, end_right;
                bool start_right_set = false, end_right_set = false;

                for (size_t i = ll.rightBound().size() - 1; i > 0; --i)
                {
                    double segment_length = std::sqrt(
                        std::pow(ll.rightBound()[i].x() - ll.rightBound()[i - 1].x(), 2) +
                        std::pow(ll.rightBound()[i].y() - ll.rightBound()[i - 1].y(), 2));

                    if (accumulated_length + segment_length > start_dist && !start_right_set)
                    {
                        double ratio = (start_dist - accumulated_length) / segment_length;
                        start_right.x = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i - 1].x() - ll.rightBound()[i].x());
                        start_right.y = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i - 1].y() - ll.rightBound()[i].y());
                        start_right_set = true;
                    }

                    if (accumulated_length + segment_length > end_dist && !end_right_set)
                    {
                        double ratio = (end_dist - accumulated_length) / segment_length;
                        end_right.x = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i - 1].x() - ll.rightBound()[i].x());
                        end_right.y = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i - 1].y() - ll.rightBound()[i].y());
                        end_right_set = true;
                        break;
                    }

                    accumulated_length += segment_length;
                }

                if (start_right_set && end_right_set)
                {
                    stripe_polygon.points.push_back(end_right);
                    stripe_polygon.points.push_back(start_right);
                }

                // Close the polygon by adding the first point again
                if (stripe_polygon.points.size() >= 4)
                {
                    stripe_polygon.points.push_back(stripe_polygon.points[0]);
                    crosswalk_polygons.polygons.push_back(stripe_polygon);
                }
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

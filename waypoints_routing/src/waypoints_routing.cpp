#include "waypoints_routing.h"

waypoints_routing::waypoints_routing(/* args */) : Node("waypoints_routing_node")
{
    this->declare_parameter("map_path", "/home/atakan/Downloads/Town10.osm");
    if (!readParameters())
        rclcpp::shutdown();

    waypoints_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_routing", 10);

    // Timer to publish periodically
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&waypoints_routing::publishWaypoints, this));

    // Load the lanelet map
    lanelet::Origin origin({49, 8.4});
    lanelet::projection::LocalCartesianProjector projector(origin);
    lanelet::LaneletMapPtr map = lanelet::load(map_path_, projector);

    for (auto &point : map->pointLayer)
    {
        point.x() = point.attribute("local_x").asDouble().value();
        point.y() = point.attribute("local_y").asDouble().value();
    }

    lanelet_routing_test(map);
}

waypoints_routing::~waypoints_routing()
{
}

bool waypoints_routing::readParameters()
{
    if (!this->get_parameter("map_path", map_path_))
    {
        std::cout << "Failed to read parameter 'map_path' " << std::endl;
        return false;
    }
    return true;
}

void waypoints_routing::lanelet_routing_test(lanelet::LaneletMapPtr &map)
{
    traffic_rules::TrafficRulesPtr trafficRules =
        traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);

    routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map, *trafficRules);

    if (routingGraph)
    {
        std::cout << green << "Routing graph built successfully" << reset << std::endl;

        lanelet::ConstLanelet startLanelet = map->laneletLayer.get(7);
        lanelet::ConstLanelet endLanelet = map->laneletLayer.get(721);

        // Check if the goal lanelet is reachable from the start lanelet
        double maxRoutingCost = 500.0;
        auto reachableSet = routingGraph->reachableSet(startLanelet, maxRoutingCost);
        bool isReachable = std::find_if(reachableSet.begin(), reachableSet.end(),
                                        [&](const lanelet::ConstLanelet &ll)
                                        { return ll.id() == endLanelet.id(); }) != reachableSet.end();

        // cout isReachabl in blue
        if (!isReachable)
        {
            std::cout << red << "Goal lanelet is not reachable from the start lanelet." << reset << std::endl;
        }
        else
        {
            std::cout << green << "Goal lanelet is reachable from the start lanelet." << reset << std::endl;
            Optional<routing::Route> route = routingGraph->getRoute(startLanelet, endLanelet, 0);
            if (route)
            {
                std::cout << green << "Route found" << reset << std::endl;

                // provides the shortest path between the startLanelet and endLanelet within the route.
                routing::LaneletPath shortestPath = route->shortestPath();

                int waypoint_id = 0;
                for (const auto &lanelet : shortestPath)
                {
                    auto points = lanelet.centerline2d();
                    for (size_t i = 0; i < points.size() - 1; ++i)
                    {

                        const auto &point = points[i];
                        const auto &next_point = points[i + 1];

                        // Calculate yaw based on difference between current and next point
                        double dx = next_point.x() - point.x();
                        double dy = next_point.y() - point.y();
                        double yaw = std::atan2(dy, dx);

                        visualization_msgs::msg::Marker waypoint_marker;
                        waypoint_marker.header.frame_id = "map";
                        waypoint_marker.header.stamp = this->now();
                        waypoint_marker.ns = "graph_waypoints";
                        waypoint_marker.id = waypoint_id++;
                        waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
                        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;

                        waypoint_marker.color.a = 1.0;

                        waypoint_marker.scale.x = 1.0;
                        waypoint_marker.scale.y = 0.5;
                        waypoint_marker.scale.z = 0.5;
                        waypoint_marker.color.r = 0.0;
                        waypoint_marker.color.g = 0.0;
                        waypoint_marker.color.b = 1.0;

                        // Set the position for the waypoint
                        waypoint_marker.pose.position.x = point.x();
                        waypoint_marker.pose.position.y = point.y();
                        waypoint_marker.pose.position.z = 0.0;

                        // Set the orientation based on the calculated yaw
                        tf2::Quaternion quaternion;
                        quaternion.setRPY(0, 0, yaw); // Roll and pitch are 0 for a flat map
                        waypoint_marker.pose.orientation.x = quaternion.x();
                        waypoint_marker.pose.orientation.y = quaternion.y();
                        waypoint_marker.pose.orientation.z = quaternion.z();
                        waypoint_marker.pose.orientation.w = quaternion.w();

                        // Add the marker to the marker array
                        graph_waypoint_markers.markers.push_back(waypoint_marker);
                    }
                }
                waypoints_publisher_->publish(graph_waypoint_markers);

                std::cout << green << "Shortest path found" << reset << std::endl;

                // graph_waypoint_markers.markers.clear();

                // returns the entire sequence of connected lanelets within the same lane as startLanelet.
                LaneletSequence fullLane = route->fullLane(startLanelet);
            }
        }
    }
}

void waypoints_routing::publishWaypoints()
{
    waypoints_publisher_->publish(graph_waypoint_markers);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<waypoints_routing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
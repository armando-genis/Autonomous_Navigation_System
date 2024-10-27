void OsmVisualizer::lanelet_routing_test(lanelet::LaneletMapPtr &map)
{
    // Create traffic rules for the routing graph
    auto trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);

    // Create routing costs container without directly constructing RoutingCostTravelTime
    lanelet::routing::RoutingCostPtrs routingCosts = lanelet::routing::RoutingCostPtrs();

    // Build the routing graph with the traffic rules and routing costs
    lanelet::routing::RoutingGraphPtr routingGraph = lanelet::routing::RoutingGraph::build(*map, *trafficRules, routingCosts);

    if (routingGraph)
    {
        const auto &lanelets = map->laneletLayer;
        if (lanelets.size() >= 2)
        {
            lanelet::ConstLanelet startLanelet = *(lanelets.begin());
            lanelet::ConstLanelet goalLanelet = *(++lanelets.begin());

            // Get the route between start and goal lanelets
            auto routeOptional = routingGraph->getRoute(startLanelet, goalLanelet);
            if (routeOptional)
            {
                const lanelet::routing::Route &route = *routeOptional;
                auto path = route.shortestPath();
                if (!path.empty())
                {
                    for (const auto &ll : path)
                    {
                        RCLCPP_INFO(this->get_logger(), "Lanelet ID in path: %ld", ll.id());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "No valid path found!");
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Route could not be found!");
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create RoutingGraph");
    }
}

// =================================================================================================

void OsmVisualizer::lanelet_routing_test(lanelet::LaneletMapPtr &map)
{
    // Create traffic rules and routing graph
    auto trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    lanelet::routing::RoutingCostPtrs routingCosts = lanelet::routing::RoutingCostPtrs();
    lanelet::routing::RoutingGraphPtr routingGraph = lanelet::routing::RoutingGraph::build(*map, *trafficRules, routingCosts);

    if (routingGraph)
    {
        const auto &lanelets = map->laneletLayer;
        if (lanelets.size() >= 2)
        {
            lanelet::ConstLanelet startLanelet = map->laneletLayer.get(793);
            lanelet::ConstLanelet goalLanelet = map->laneletLayer.get(35);

            // cout the lanelet id in red color
            std::cout << red << "----> Start Lanelet ID: " << red << startLanelet.id() << reset << std::endl;
            std::cout << red << "----> Goal Lanelet ID: " << red << goalLanelet.id() << reset << std::endl;

            visualization_msgs::msg::MarkerArray graph_waypoint_markers;
            int waypoint_id = 0;

            // Iterate over all lanelets in the map to extract all waypoints in the graph
            for (const auto &lanelet : lanelets)
            {
                for (const auto &point : lanelet.centerline2d())
                {
                    visualization_msgs::msg::Marker waypoint_marker;
                    waypoint_marker.header.frame_id = "map";
                    waypoint_marker.header.stamp = rclcpp::Clock{}.now();
                    waypoint_marker.ns = "graph_waypoints";
                    waypoint_marker.id = waypoint_id++;
                    waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
                    waypoint_marker.action = visualization_msgs::msg::Marker::ADD;

                    waypoint_marker.color.a = 1.0;

                    // Set color based on whether the point belongs to the start or goal lanelet
                    if (&lanelet == &startLanelet || &lanelet == &goalLanelet)
                    {
                        // Red color for start and goal nodes
                        waypoint_marker.scale.x = 0.9;
                        waypoint_marker.scale.y = 0.9;
                        waypoint_marker.scale.z = 0.9;

                        waypoint_marker.color.r = 1.0;
                        waypoint_marker.color.g = 0.0;
                        waypoint_marker.color.b = 0.0;
                        // std::cout << "----> Lanelet ID: " << lanelet.id() << std::endl;
                    }
                    else
                    {
                        // Blue color for other nodes
                        waypoint_marker.scale.x = 0.3;
                        waypoint_marker.scale.y = 0.3;
                        waypoint_marker.scale.z = 0.3;
                        waypoint_marker.color.r = 0.0;
                        waypoint_marker.color.g = 0.0;
                        waypoint_marker.color.b = 1.0;

                        // print the lanelet id in blue color
                        std::cout << blue << "----> Lanelet ID: " << blue << lanelet.id() << reset << std::endl;
                    }

                    // Set the position for the waypoint
                    waypoint_marker.pose.position.x = point.x();
                    waypoint_marker.pose.position.y = point.y();
                    waypoint_marker.pose.position.z = 0.0;

                    // Add the marker to the marker array
                    graph_waypoint_markers.markers.push_back(waypoint_marker);
                }
            }

            // Publish the complete graph's waypoints as a MarkerArray
            waypoints_publisher_->publish(graph_waypoint_markers);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Not enough lanelets to define start and goal nodes.");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create RoutingGraph");
    }
}
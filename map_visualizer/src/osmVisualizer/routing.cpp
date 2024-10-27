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

    // Add a routing cost module with parameters for laneChangeCost and minLaneChangeLength
    double laneChangeCost = 10.0;     // Example cost for lane changes
    double minLaneChangeLength = 5.0; // Example minimum lane change length
    lanelet::routing::RoutingCostPtrs routingCosts = {std::make_shared<lanelet::routing::RoutingCostDistance>(laneChangeCost, minLaneChangeLength)};

    // Build the routing graph with the specified routing costs
    lanelet::routing::RoutingGraphPtr routingGraph = lanelet::routing::RoutingGraph::build(*map, *trafficRules, routingCosts);

    if (routingGraph)
    {
        const auto &lanelets = map->laneletLayer;

        // Retrieve start and goal lanelets by ID
        lanelet::ConstLanelet startLanelet = map->laneletLayer.get(77);
        lanelet::ConstLanelet goalLanelet = map->laneletLayer.get(512);

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
                if (lanelet.id() == startLanelet.id())
                {
                    // Red color for start and goal nodes
                    waypoint_marker.scale.x = 1.7;
                    waypoint_marker.scale.y = 1.7;
                    waypoint_marker.scale.z = 1.7;
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 0.0;

                    std::cout << yellow << "----> Lanelet ID: " << blue << lanelet.id() << reset << std::endl;
                }
                else if (lanelet.id() == goalLanelet.id())
                {
                    // Green color for other nodes
                    waypoint_marker.scale.x = 1.7;
                    waypoint_marker.scale.y = 1.7;
                    waypoint_marker.scale.z = 1.7;
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 1.0;
                    waypoint_marker.color.b = 0.0;

                    std::cout << green << "----> Lanelet ID: " << blue << lanelet.id() << reset << std::endl;
                }
                else
                {
                    // Blue color for other nodes
                    waypoint_marker.scale.x = 1.0;
                    waypoint_marker.scale.y = 1.0;
                    waypoint_marker.scale.z = 1.0;
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 1.0;

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

        // Find the shortest path between the start and goal lanelets
        auto route = routingGraph->getRoute(startLanelet, goalLanelet);
        if (route)
        {
            std::cout << "Route found with " << route->shortestPath().size() << " lanelets." << std::endl;
            for (const auto &lanelet : route->shortestPath())
            {
                std::cout << "Lanelet ID: " << lanelet.id() << std::endl;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No route found between start and goal lanelets.");
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

    // Set routing costs and configuration
    double laneChangeCost = 5.0;
    double minLaneChangeLength = 0.0;
    lanelet::routing::RoutingCostPtrs routingCosts = {std::make_shared<lanelet::routing::RoutingCostDistance>(laneChangeCost, minLaneChangeLength)};
    lanelet::routing::RoutingGraph::Configuration routingGraphConf;
    routingGraphConf.emplace(std::make_pair(lanelet::routing::RoutingGraph::ParticipantHeight, lanelet::Attribute("2.")));

    // Build the routing graph with configuration and costs
    lanelet::routing::RoutingGraphPtr routingGraph = lanelet::routing::RoutingGraph::build(*map, *trafficRules /*, routingCosts, routingGraphConf*/);

    if (routingGraph)
    {
        const auto &lanelets = map->laneletLayer;

        // Retrieve start and goal lanelets by ID
        lanelet::ConstLanelet startLanelet = map->laneletLayer.get(28);
        lanelet::ConstLanelet goalLanelet = map->laneletLayer.get(56);

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
                if (lanelet.id() == startLanelet.id())
                {
                    // Red color for start and goal nodes
                    waypoint_marker.scale.x = 1.7;
                    waypoint_marker.scale.y = 1.7;
                    waypoint_marker.scale.z = 1.7;
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 0.0;

                    std::cout << red << "----> Lanelet ID: " << blue << lanelet.id() << reset << std::endl;
                }
                else if (lanelet.id() == goalLanelet.id())
                {
                    // Green color for other nodes
                    waypoint_marker.scale.x = 1.7;
                    waypoint_marker.scale.y = 1.7;
                    waypoint_marker.scale.z = 1.7;
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 1.0;
                    waypoint_marker.color.b = 0.0;

                    std::cout << green << "----> Lanelet ID: " << blue << lanelet.id() << reset << std::endl;
                }
                else
                {
                    // Blue color for other nodes
                    waypoint_marker.scale.x = 1.0;
                    waypoint_marker.scale.y = 1.0;
                    waypoint_marker.scale.z = 1.0;
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 1.0;

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

        // Check reachability of the goal lanelet from the start lanelet
        double maxRoutingCost = 2000.0;

        auto shortestPath = routingGraph->shortestPath(startLanelet, goalLanelet);
        auto route_fist = routingGraph->getRoute(startLanelet, goalLanelet);

        if (route_fist)
        {
            std::cout << green << "-----> Route found" << reset << std::endl;
        }
        else
        {
            std::cout << red << "-----> Route not found" << reset << std::endl;
        }

        auto reachableSet = routingGraph->reachableSet(startLanelet, maxRoutingCost);
        bool isReachable = std::find_if(reachableSet.begin(), reachableSet.end(),
                                        [&](const lanelet::ConstLanelet &ll)
                                        { return ll.id() == goalLanelet.id(); }) != reachableSet.end();

        if (!isReachable)
        {
            RCLCPP_WARN(this->get_logger(), "Goal lanelet is not reachable from the start lanelet.");
        }
        else
        {
            // Proceed with route finding if reachable
            auto route = routingGraph->getRoute(startLanelet, goalLanelet);
            if (route)
            {
                std::cout << "Route found with " << route->shortestPath().size() << " lanelets." << std::endl;
                for (const auto &lanelet : route->shortestPath())
                {
                    std::cout << "Lanelet ID in route: " << lanelet.id() << std::endl;
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No route found between start and goal lanelets.");
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create RoutingGraph");
    }
}

// auto adjacentLeft = routingGraph->adjacentLeft(startLanelet);
// auto adjacentRight = routingGraph->adjacentRight(startLanelet);
// auto following = routingGraph->following(startLanelet);
// auto previous = routingGraph->previous(startLanelet);

// std::cout << "Lanelet 14 adjacent left: " << (adjacentLeft ? adjacentLeft->id() : -1) << std::endl;
// std::cout << "Lanelet 14 adjacent right: " << (adjacentRight ? adjacentRight->id() : -1) << std::endl;
// std::cout << "Lanelet 14 following lanelets: ";
// for (const auto &ll : following)
// {
//   std::cout << ll.id() << " ";
// }
// std::cout << std::endl;
// std::cout << "Lanelet 14 previous lanelets: ";
// for (const auto &ll : previous)
// {
//   std::cout << ll.id() << " ";
// }
// std::cout << std::endl;
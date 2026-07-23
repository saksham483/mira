#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "common.hpp"
#include "conditions/conditions.hpp"
#include "actions/actions.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mira2_actions_node");

    std::string default_xml = ament_index_cpp::get_package_share_directory("mira2_actions") + "/config/sauvc_mission.xml";
    node->declare_parameter("tree_xml", default_xml);
    std::string tree_xml_path = node->get_parameter("tree_xml").as_string();

    ROSState ros_state(node);
    auto vision_tracker = std::make_shared<VisionTracker>(&ros_state);

    BT::BehaviorTreeFactory factory;

    // Conditions
    factory.registerNodeType<IsEmergency>("IsEmergency", &ros_state);
    factory.registerNodeType<IsTaskEnabled>("IsTaskEnabled");
    factory.registerNodeType<DetectImpact>("DetectImpact", &ros_state);
    
    // Actions
    factory.registerNodeType<DiveToDepth>("DiveToDepth",&ros_state);
    factory.registerNodeType<Search>("Search", &ros_state,vision_tracker);
    factory.registerNodeType<TrackAndApproach>("TrackAndApproach",&ros_state,vision_tracker);
    factory.registerNodeType<BlindMove>("BlindMove",&ros_state);
    factory.registerNodeType<ActuateMechanism>("ActuateMechanism",&ros_state);
    factory.registerNodeType<EmergencySurface>("EmergencySurface",&ros_state);
    factory.registerNodeType<SetFlightMode>("SetFlightMode",&ros_state);
    auto tree = factory.createTreeFromFile(tree_xml_path);
    BT::Groot2Publisher publisher(tree, 1337);
    
    RCLCPP_INFO(node->get_logger(), "Behavior Tree executing. Groot2 on port 1337");

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        tree.tickOnce();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
#include "mep3_behavior/bt_topic_sub_node.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <iostream>
#include <vector>

using namespace BT;
namespace mep3_behavior
{
  class CameraDetection : public RosTopicSubNode<std_msgs::msg::Int32MultiArray>
  {
  public:
    CameraDetection(const std::string &name,
                    const NodeConfig &conf,
                    const RosNodeParams &params)
        : RosTopicSubNode<std_msgs::msg::Int32MultiArray>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {

      return {BT::InputPort<int>("plant_position")};
    }

    NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Int32MultiArray> &last_msg) override
    {
      if (last_msg == nullptr)
      {
        return NodeStatus::FAILURE;
      }

      int plant_position;
      bool is_plant_detected;
      getInput<int>("plant_position", plant_position);

      std::vector<int> detection_results = last_msg->data;

      if (detection_results.at(plant_position - 1) == 0)
        is_plant_detected = false;
      else
        is_plant_detected = true;

      std::cout << "Detection result " << detection_results.at(plant_position - 1) << " at position: " << plant_position << std::endl;

      if (is_plant_detected == true)
        return NodeStatus::SUCCESS;

      return NodeStatus::FAILURE;
    }
  };
}

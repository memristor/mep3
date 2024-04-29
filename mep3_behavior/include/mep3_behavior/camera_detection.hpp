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

      return {BT::InputPort<int>("weed_position")};
    }

    NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Int32MultiArray> &last_msg) override
    {
      if (last_msg == nullptr)
      {
        return NodeStatus::FAILURE;
      }

      int weed_position;
      getInput<int>("weed_position", weed_position);
      std::cout << "-------------------------------" << weed_position << std::endl;

      std::vector<int> detection_results = last_msg->data;

      std::cout << detection_results.at(weed_position - 1) << std::endl;

      // if (last_msg == nullptr)
      //   return NodeStatus::SUCCESS;

      // if (last_msg->data == true)
      //   return NodeStatus::FAILURE;

      return NodeStatus::SUCCESS;
    }
  };
}

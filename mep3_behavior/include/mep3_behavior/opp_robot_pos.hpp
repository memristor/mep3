#ifndef MEP3_BEHAVIOR_TREE__OPP_ROBOT_POS_HPP
#define MEP3_BEHAVIOR_TREE__OPP_ROBOT_POS_HPP

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <string>
#include <bitset>

namespace mep3_behavior{
    class OppRobotInZone : public BT::ConditionNode{
        public:
            OppRobotInZone(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config){
                if(!getInput<int>("zone_select", zone_select_)){
                    throw BT::RuntimeError("Missing argument zone_select!");
                }
                if(zone_select_ > 8)
                    throw BT::RuntimeError("Argument zone_select must be less than or queal to 8!");
            }

            static BT::PortsList providedPorts(){
                BT::PortsList port_list = {
                    BT::InputPort<int>("zone_select")
                };

                return port_list;
            }

            BT::NodeStatus tick() override{
                auto blackboard = BT::SharedBlackboard::access();
                
                std::string robot_pos;
                robot_pos = blackboard->get<std::string>("opp_robot_zone");
                    //throw BT::RuntimeError("Blackboard error in zone_value!");
                    //return BT::NodeStatus::FAILURE;
                
                
                int temp = stoi(robot_pos);
                uint8_t zone = static_cast<uint8_t>(zone_select_);
                uint8_t mask = 0x01 << (zone - 1);

                std::cout << "Opponent robot is in zone: " << " " << std::bitset<8>(temp) << std::endl;
                std::cout << "Mask value is: " << " " << std::bitset<8>(mask) << std::endl;

                if((temp & mask) > 0){
                    return BT::NodeStatus::SUCCESS;
                }else{
                    return BT::NodeStatus::FAILURE;
                }
            }

        private:
            int zone_select_;
    };
};

#endif
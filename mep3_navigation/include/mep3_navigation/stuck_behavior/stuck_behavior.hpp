#ifndef MEP3_NAVIGATION__STUCK_BEHAVIOR_HPP_
#define MEP3_NAVIGATION__STUCK_BEHAVIOR_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "mep3_msgs/action/move.hpp"
#include "mep3_msgs/msg/motion_properties.hpp"
#include "nav2_util/node_utils.hpp"
#include "ruckig/ruckig.hpp"
#include "mep3_navigation/stuck_detector.hpp"

namespace mep3_navigation
{
  using ActionT = mep3_msgs::action::Move;

  class StuckBehavior : public nav2_behaviors::TimedBehavior<ActionT>
  {
  public:
    StuckBehavior();
    ~StuckBehavior();

    virtual nav2_behaviors::Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) final override;

    nav2_behaviors::Status onCycleUpdate();

  protected:
  };
}

#endif // MEP3_NAVIGATION__STUCK_BEHAVIOR_HPP_

#ifndef GOAL_TABLES
#define GOAL_TABLES \
    BT::InputPort<std::string>("goal_foo"), \
    BT::InputPort<std::string>("goal_bar")
#endif

#ifndef POSITION_TABLES
#define POSITION_TABLES \
    BT::InputPort<std::string>("position_foo"), \
    BT::InputPort<std::string>("position_bar")
#endif

#ifndef RESISTANCE_TABLES
#define RESISTANCE_TABLES \
    BT::InputPort<std::string>("resistance_foo"), \
    BT::InputPort<std::string>("resistance_bar")
#endif

#ifndef VALUE_TABLES
#define VALUE_TABLES \
    BT::InputPort<std::string>("value_foo"), \
    BT::InputPort<std::string>("value_bar")
#endif

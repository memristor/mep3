// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Connector.hpp>
#include <algorithm>
#include <iostream>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  
  Motor *m0 = robot->getMotor("robot_arm0_motor_base");
  Motor *m1 = robot->getMotor("robot_arm0_motor_mid");
  Motor *m2 = robot->getMotor("robot_arm0_motor_gripper");
  PositionSensor *ps0 = robot->getPositionSensor("robot_arm0_sensor_base");
  PositionSensor *ps1 = robot->getPositionSensor("robot_arm0_sensor_mid");
  PositionSensor *ps2 = robot->getPositionSensor("robot_arm0_sensor_gripper");
  ps0->enable(timeStep);
  ps1->enable(timeStep);
  ps2->enable(timeStep);
  Connector *c_arm = robot->getConnector("connector");
  c_arm->enablePresence(timeStep);
  m0->enableTorqueFeedback(timeStep);
  m1->enableTorqueFeedback(timeStep);
  m2->enableTorqueFeedback(timeStep);
  int printv = 0;
  double max_torque0, max_torque1, max_torque2;
  max_torque0 = max_torque1 = max_torque2 = 0;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    max_torque0 = std::max(max_torque0, m0->getTorqueFeedback());
    max_torque1 = std::max(max_torque1, m1->getTorqueFeedback());
    max_torque2 = std::max(max_torque2, m2->getTorqueFeedback());
    std::cout << "max torque0 = " << max_torque0 << ", curr torque0 = " << m0->getTorqueFeedback() << std::endl;
    std::cout << "max torque1 = " << max_torque1 << ", curr torque1 = " << m1->getTorqueFeedback() << std::endl;
    std::cout << "max torque2 = " << max_torque2 << ", curr torque2 = " << m2->getTorqueFeedback() << std::endl;

    //std::cout << "presence: " << c_arm->getPresence() << std::endl;
    if (!printv) {
      std::cout << "PS: " << ps0->getValue() << " " << ps1->getValue() << " " << ps2->getValue() << std::endl; 
    }
    else 
      std::cout << "CONNECTED" << std::endl;
    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    if (!(c_arm->getPresence())) {
      m0->setPosition(-1.5708);
      m1->setPosition(0.0);
      m2->setPosition(-1.5708);
    }
    else {
      c_arm->lock();
      m0->setPosition(0);
      printv=1;
    }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

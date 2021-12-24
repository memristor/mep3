// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Connector.hpp>
#include <algorithm>
#include <iostream>

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  Motor *m0 = robot->getMotor("test_arm_motor_base");
  Motor *m1 = robot->getMotor("test_arm_motor_mid");
  Motor *m2 = robot->getMotor("test_arm_motor_gripper");
  PositionSensor *ps0 = robot->getPositionSensor("test_arm_sensor_base");
  PositionSensor *ps1 = robot->getPositionSensor("test_arm_sensor_mid");
  PositionSensor *ps2 = robot->getPositionSensor("test_arm_sensor_gripper");
  ps0->enable(timeStep);
  ps1->enable(timeStep);
  ps2->enable(timeStep);
  Connector *c_arm = robot->getConnector("test_arm_connector");
  c_arm->enablePresence(timeStep);
  m0->enableTorqueFeedback(timeStep);
  m1->enableTorqueFeedback(timeStep);
  m2->enableTorqueFeedback(timeStep);
  int printv = 0;
  double max_torque0, max_torque1, max_torque2;
  max_torque0 = max_torque1 = max_torque2 = 0;
  bool measure = false;
  while (robot->step(timeStep) != -1) {
    // Read the sensors:

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
      if (measure) {
      max_torque0 = std::max(max_torque0, std::abs(m0->getTorqueFeedback()));
      max_torque1 = std::max(max_torque1, std::abs(m1->getTorqueFeedback()));
      max_torque2 = std::max(max_torque2, std::abs(m2->getTorqueFeedback()));
      }
    // Enter here functions to send actuator commands, like:
    if (!(c_arm->getPresence())) {
      m0->setPosition(-1.5708);
      m1->setPosition(1.5708);
      m2->setPosition(1.5708);
    }
    else {
      c_arm->lock();
      m0->setPosition(0);
      m1->setPosition(0);
      m2->setPosition(0);
      if (printv)       
        measure = true;
      printv=1;
      
    }
  };

  delete robot;
  return 0;
}
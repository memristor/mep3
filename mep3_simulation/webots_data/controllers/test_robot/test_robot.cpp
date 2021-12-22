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

/*
   ABBREVIATIONS:
   lam - Left Arm Motor
   ram - Right Arm Motor
   laps - Left Arm Position Sensor
   raps - Right Arm Position Sensor
   lwm - Left Wheel Motor
   rwm - Right Wheel Motor
   leps - Left Encoder Position Sensor
   reps - Right Encoder Position Sensor
*/

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // Motors
  Motor *lwm = robot->getMotor("left_motor");
  Motor *rwm = robot->getMotor("right_motor");
  PositionSensor *leps = robot->getPositionSensor("test_robot_encoder_left");
  PositionSensor *reps = robot->getPositionSensor("test_robot_encoder_right");
  leps->enable(timeStep);
  reps->enable(timeStep);
  
  // Left Arm
  Motor *lam0 = robot->getMotor("test_robot_arm_left_motor_base");
  Motor *lam1 = robot->getMotor("test_robot_arm_left_motor_mid");
  Motor *lam2 = robot->getMotor("test_robot_arm_left_motor_gripper");
  PositionSensor *laps0 = robot->getPositionSensor("test_robot_arm_left_sensor_base");
  PositionSensor *laps1 = robot->getPositionSensor("test_robot_arm_left_sensor_mid");
  PositionSensor *laps2 = robot->getPositionSensor("test_robot_arm_left_sensor_gripper");
  laps0->enable(timeStep);
  laps1->enable(timeStep);
  laps2->enable(timeStep);
  Connector *c_arm = robot->getConnector("test_robot_arm_left_connector");
  c_arm->enablePresence(timeStep);
  
  // pause
  for (int i = 0; i < 20; i++)
    robot->step(timeStep);
  
  // states
  enum States {translate, rotate, translate_back, grab, raise};
  States state = translate;
  while (robot->step(timeStep) != -1) {
    switch (state){
      case translate:
        lwm->setPosition(-10);
        rwm->setPosition(-10);
        //std::cout << leps->getValue() << std::endl;
        //std::cout << reps->getValue() << std::endl;
        if (leps->getValue() < -9.9)
          state = rotate;
        break;
      case rotate:
        //lwm->setPosition(-10);
        //rwm->setPosition(+10);
        state = translate_back;
        break;
      case translate_back:
        lwm->setPosition(10);
        rwm->setPosition(10);
        // std::cout << leps->getValue() << std::endl;
        // std::cout << reps->getValue() << std::endl;
        if (leps->getValue() >= 0)
          state = grab;
        break;
      case grab:
        if (!(c_arm->getPresence())) {
          lam0->setVelocity(0.2);
          lam0->setPosition(1.5708);
          lam1->setVelocity(0.2);
          lam1->setPosition(1.5708);
          lam2->setPosition(1.5708);
        }
        else {
          state = raise;
        }
        break;
      case raise:
        c_arm->lock();
        //lam0->setPosition(0);
        lam1->setPosition(0.5);
        //lam2->setPosition(0);
        break;
      default:
        break;
    } 
    // Read the sensors:

    // Enter here functions to send actuator commands, like:
    
  };

  delete robot;
  return 0;
}
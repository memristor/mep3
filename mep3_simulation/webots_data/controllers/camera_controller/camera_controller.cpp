#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#define TIME_STEP 64

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Camera *cm;
  cm=robot->getCamera("RasPi0");
  cm->enable(timeStep);
  //std::cout << "Focal length = " << cm->getFocalLength() << std::endl;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();


    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };
      cm->saveImage("../../src/img/top_yellow.png", 100);

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

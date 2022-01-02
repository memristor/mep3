// File:          my_opponent_1.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Supervisor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;


int main() {
  // create the Robot instance.
   Supervisor *supervisor = new Supervisor();
   
   Node *opponent_robot= supervisor->getFromDef("opponent_robot_1");
   Field *opponet_field_1=opponent_robot->getField("translation");

  // get the time step of the current world.
   int timeStep = (int)supervisor->getBasicTimeStep();

  
    double pos[3]={0,0,0};
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (supervisor->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    const double *opponent_trans=opponet_field_1->getSFVec3f();
    pos[0]=opponent_trans[0];
    pos[1]=opponent_trans[1];
    pos[2]=opponent_trans[2];
    
    
    opponet_field_1->setSFVec3f(pos);
   
  };

  // Enter here exit cleanup code.

  delete supervisor;
  return 0;
}

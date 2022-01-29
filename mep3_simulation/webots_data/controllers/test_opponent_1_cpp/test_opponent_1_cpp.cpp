#include <webots/Supervisor.hpp>
 
 
using namespace webots;
 
int main() {
  Supervisor *supervisor = new Supervisor();
 
  Node *opponent_robot= supervisor->getSelf();
  Field *opponet_field_1=opponent_robot->getField("translation");
 
  int timeStep = (int)supervisor->getBasicTimeStep();
  double pos[3]={0,0,0};
 
  while (supervisor->step(timeStep) != -1) {
    const double *opponent_trans=opponet_field_1->getSFVec3f();
     pos[0]=opponent_trans[0];
     pos[1]=opponent_trans[1];
     pos[2]=opponent_trans[2];
    opponet_field_1->setSFVec3f(pos);
  };
 
  delete supervisor;
  return 0;
}
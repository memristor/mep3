from controller import Supervisor
# create the Robot instance.
supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
opponent_node=supervisor.getSelf()
opponent_filed=opponent_node.getField('translation')

pos=[0,0,0]
# Main loop:

while supervisor.step(timestep) != -1:
    values = opponent_filed.getSFVec3f()
    pos[0]=values[0]
    pos[1]=values[1]
    pos[2]=values[2]
    
    opponent_filed.setSFVec3f(pos)

    pass

# Enter here exit cleanup code.

from controller import Supervisor
# create the Robot instance.
import random


supervisor = Supervisor()
 
# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
opponent_node=supervisor.getSelf()
name_field = opponent_node.getField('name')
opponent_field=opponent_node.getField('translation')
name = name_field.getSFString()

position=[0,0,0]

positions=[(-0.97, 0.40, 10),(-1.17, 0.69, 10),
           (-0.72, 0.55, 15), (-0.4, 0.68, 20), 
           (-0.77, 0.26, 15),(-0.9, -0.43, 15),
           (-0.62, 0.43, 5),(-1.21, -0.22, 15),
           (-1.12, -0.62, 20),(-0.65,-0.73,10), 
           (-1.3, 0.13, 10), (-1.29, 0.46, 10)]
           

def destination_achieved(curr, dest, epsilon):
    return (abs(curr[0]-dest[0])<epsilon and abs(curr[1]-dest[1])<epsilon)
      
 
def time_achieved(time_period):
   ret_value=False
   t=supervisor.getTime()
   while supervisor.getTime()-t<time_period:
      supervisor.step(timestep)
      ret_value=False
   else:
      ret_value=True
                 
   return  ret_value
   

delta=0.001
epsilon=0.05
achieved_destination=False
destination=positions[random.randint(0,len(positions)-1)]
while supervisor.step(timestep)!= -1:
    
    current_position = opponent_field.getSFVec3f()
    position[0]= current_position[0]
    position[1]= current_position[1]
    position[2]= current_position[2]
    
  
    if(achieved_destination):
       destination=random.choice(positions)
       
    
    if (destination_achieved(current_position, destination, epsilon)):
      
      if(time_achieved(destination[2])):
        t=supervisor.getTime()    
        
        achieved_destination=True
        
    else:
        achieved_destination=False
        if position[0]<destination[0]:
            position[0]+=delta
        if position[1]<destination[1]:
             position[1]+=delta
        if position[0]>destination[0]:
             position[0]-=delta
        if position[1]>destination[1]:
             position[1]-=delta
        if(position[0]!=destination[0] and position[1]!=destination[1]):
             opponent_field.setSFVec3f(position)
             
        else:
            position[0]=destination[0]
            position[1]=destination[1]
            
               
    pass
    
    # Enter here exit cleanup code.
 
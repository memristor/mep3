from controller import Supervisor
# create the Robot instance.
import time
from datetime import datetime

 
 
supervisor = Supervisor()
 
# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
opponent_node=supervisor.getSelf()
name_field = opponent_node.getField('name')
opponent_filed=opponent_node.getField('translation')
name = name_field.getSFString()
pos=[0,0,0]

positions=[(-0.97, 0.40, 0.175), (-1.5, 0.5, 0.175),
           (-0.97, 0.64, 0.175), (-1.23, 0.49, 0.175), 
           (-0.62, 0.49, -0.175),(-0.62, 0.57, 0.175),
           (-0.62, 0.43, 0.175), (-1.29, 0.46, 0.175)]
           
time=[10,10,15,20,10, 5,10,25]
def destination_achieved(curr, dest, epsilon):
    if (abs(curr[0]-dest[0])<epsilon and abs(curr[1]-dest[1])<epsilon):
        return True
    else:
        return False
 
# def time_achieved(time,pos,dest):
   # if(abs(pos[0]-dest[0])<0.05 and abs(pos[1]-dest[1])<0.05):
     # t=datetime.now().second
     # ret=False
     # if(t<time):
         # t=datetime.now().second
     # else:
          # ret=True
   # return ret

i = 0
while supervisor.step(timestep) != -1:
   
    curr = opponent_filed.getSFVec3f()
    pos[0]= curr[0]
    pos[1]= curr[1]
    pos[2]= curr[2]
    
    
    if(i<len(positions)):
        dest = positions[i]
        # t=time[i]
        
    
    if (destination_achieved(curr, dest, 0.05) == True):
     # if(time_achieved(t,pos,dest)==True):
        i = i+1
        if (name == 'opponent_box'):
            print (dest)
    else:
        if pos[0]<dest[0]:
            pos[0]+=0.001
        if pos[1]<dest[1]:
             pos[1]+=0.001
        if pos[0]>dest[0]:
             pos[0]-=0.001
        if pos[1]>dest[1]:
             pos[1]-=0.001
        if(pos[0]!=dest[0] and pos[1]!=dest[1]):
             opponent_filed.setSFVec3f(pos)
        # if(abs(pos[0]-dest[0])<0.05 and abs(pos[1]-dest[1])<0.05):
               # j=0
               # j+=1
               # print(j)
                   
        else:
            pos[0]=dest[0]
            pos[1]=dest[1]
    # time.sleep(1)
               
    
    pass
    
    # Enter here exit cleanup code.
 
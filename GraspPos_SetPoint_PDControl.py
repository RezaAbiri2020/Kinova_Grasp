import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import pygame as pg
import random
import numpy as np
from numpy import savetxt
import os
import csv
from scipy.spatial.transform import Rotation as R

from Packages.Camera_Scripts.Camera import IntelCamera

use2D   = 0
logData = 0
# activate record or not record the simulation data/ kinematics data
SaveData = 0

if use2D == 1:
  from Packages.UITele_Scripts.cursorUITest_2D import UI
else:
  from Packages.UITele_Scripts.cursorUITest import UI

clid = p.connect(p.SHARED_MEMORY)
if (clid<0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()

p.loadURDF("plane.urdf",[0,0,-.65])
p.loadURDF("table/table.urdf", basePosition=[-0.4,0.0,-0.65])
# p.loadURDF("tray/tray.urdf",[-0.8,-0.0,0.0])

# consider of three objects on a circle
Obj1_Pos = [-0.35, -0.35, 0]
Obj2_Pos = [-0.5, 0, 0]
Obj3_Pos = [-0.35, 0.35, 0]

# these angles should be the corresponding euler angles for final end-effector grasp pose 
# possible final value for grasping obj1 [Roll, Pitch, Yaw] 
Obj1_GraspPos = [-0.36, -0.34, 0.03]
Obj1_GraspOrn = [3.0612682251627628, 1.5, 1.199751013269177]
# possible final value for grasping obj2 [Roll, Pitch, Yaw]
Obj2_Orn = [0, math.pi/2, 0]
# possible final value for grasping obj3 [Roll, Pitch, Yaw]
Obj3_Orn = [0, 0, -math.pi/2]

p.loadURDF("dinnerware/cup/cup_small.urdf",Obj1_Pos)
p.loadURDF("dinnerware/cup/cup_small.urdf",Obj2_Pos)
p.loadURDF("dinnerware/cup/cup_small.urdf",Obj3_Pos)


#p.loadURDF("dinnerware/plate.urdf",[-0.3,0,0.0])
#p.loadURDF("cube_small.urdf",[-0.4,0.35,0.0])
#p.loadURDF("sphere_small.urdf",[-0.2,-0.35,0.0])
#p.loadURDF("duck_vhacd.urdf",[0,-0.45,0.0])
#p.loadURDF("teddy_vhacd.urdf",[0.1,-0.35,0.0])

#p.loadURDF("block.urdf",[-0.7,0.0,0.0])

#cube1Id = p.loadURDF("cube_small.urdf",[-0.4,-0.4,0.0])
# p.loadURDF("cube_small.urdf",[-0.3,-0.15,0.0])
# p.loadURDF("cube_small.urdf",[-0.2,0.2,0.0])

jacoId = p.loadURDF("jaco/j2n6s300.urdf", [0,0,0],  useFixedBase=True)

basePos = [0,0,0]
p.resetBasePositionAndOrientation(jacoId,basePos,[0,0,0,1])

# to observe the robot from closer view; from a camera view
# also to record a video from this camera; uncomment the line in the method to record
NameofRecord = 'Obj1_SetPoint_PDController'
Record = 0
Camera_Class = IntelCamera(NameofRecord, Record)
Camera_Class.video()

jacoEndEffectorIndex = 8
numJoints = 10
jacoArmJoints = [2, 3, 4, 5, 6, 7]
jacoFingerJoints = [9, 11, 13]

#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# inital angle positions consider from the second one 
rp = [0, math.pi/4, math.pi, 1.0*math.pi, 1.8*math.pi, 0*math.pi, 1.75*math.pi, 0.5*math.pi]

# upper and lower bands for positioning of the end-effector
wu = [0.1, 0.5, 0.5]
wl = [-.66, -.5, 0.00]

for i in range(8):
  p.resetJointState(jacoId,i, rp[i])

ls = p.getLinkState(jacoId, jacoEndEffectorIndex)
p.setGravity(0,0,-10)

t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

useOrientation = 1
useSimulation = 1
useRealTimeSimulation = 1
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) 
trailDuration = 5

pg.init()
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d, %d" % (0,0)
pg.display.set_mode((500,500))
pg.display.set_caption("Control Interface")
runUI = UI(logData)

# positions and orientations of end-effector
pos = list(ls[4])
orn = list(ls[5])

# grab the initial positions of the end-effector for autonomous grasp
# grab the initial euler orientations of the end-effector for autonomous grasp
X_0 = pos[0]
Y_0 = pos[1]
Z_0 = pos[2]

eulOrn = p.getEulerFromQuaternion(orn)
Roll_0, Pitch_0, Yaw_0 = eulOrn 

# initiate the arrays for concatanate later and saving
#End_Effector_Pos = np.array([pos])
#End_Effector_Orn = np.array([eulOrn])

# define the possible final desired positions/orientation of the end-effector to make corresponding autonomous grasp
# object 1
X_f1 = Obj1_GraspPos[0]
Y_f1 = Obj1_GraspPos[1]
Z_f1 = Obj1_GraspPos[2]
Roll_f1 = Obj1_GraspOrn[0]
Pitch_f1 = Obj1_GraspOrn[1]
Yaw_f1 = Obj1_GraspOrn[2]
# object 2
X_f2 = Obj2_Pos[0]
Y_f2 = Obj2_Pos[1]
Z_f2 = Obj2_Pos[2]
Roll_f2 = Obj2_Orn[0]
Pitch_f2 = Obj2_Orn[1]
Yaw_f2 = Obj2_Orn[2]
# object 3
X_f3 = Obj3_Pos[0]
Y_f3 = Obj3_Pos[1]
Z_f3 = Obj3_Pos[2]
Roll_f3 = Obj3_Orn[0]
Pitch_f3 = Obj3_Orn[1]
Yaw_f3 = Obj3_Orn[2]

# define the final desired and arbitrary positions/orientations of the end-effector
Obj_Pos = [-0.35, -0.35, 0.2]
Obj_Orn = [math.pi/2, math.pi/3, 2*math.pi/3]
X_f = Obj_Pos[0]
Y_f = Obj_Pos[1]
Z_f = Obj_Pos[2]
Roll_f = Obj_Orn[0] 
Pitch_f = Obj_Orn[1]
Yaw_f = Obj_Orn[2] 


# start i for later while loop
i=0
JP = list(rp[2:9])
fing = 0
wri = 0

newPosInput = 1
keyT = time.time()
kTotal = np.zeros([9,], dtype = int)

pg.key.set_repeat()
kp5_up = 1
add_KP5 = 1

dist = .002
ang = .005
rot_theta = .008
inputRate = .05

# for rotation in forward around x, y, z 
Rx = np.array([[1., 0., 0.],[0., np.cos(rot_theta), -np.sin(rot_theta)], [0., np.sin(rot_theta), np.cos(rot_theta)]])
Ry = np.array([[np.cos(rot_theta), 0., np.sin(rot_theta)], [0., 1., 0.], [-np.sin(rot_theta), 0., np.cos(rot_theta)]])
Rz = np.array([[np.cos(rot_theta), -np.sin(rot_theta), 0.], [np.sin(rot_theta), np.cos(rot_theta), 0.], [0., 0., 1.]])

# for rotation in backward around x, y, z
Rxm = np.array([[1., 0., 0.],[0., np.cos(-rot_theta), -np.sin(-rot_theta)], [0., np.sin(-rot_theta), np.cos(-rot_theta)]])
Rym = np.array([[np.cos(-rot_theta), 0., np.sin(-rot_theta)], [0., 1., 0.], [-np.sin(-rot_theta), 0., np.cos(-rot_theta)]])
Rzm = np.array([[np.cos(-rot_theta), -np.sin(-rot_theta), 0.], [np.sin(-rot_theta), np.cos(-rot_theta), 0.], [0., 0., 1.]])

updateT = time.time()


# choose the name for recroding here
if SaveData:
  if not os.path.exists('Data'):
    os.makedirs('Data')

  DateDir = time.strftime("%Y%m%d")
  if not os.path.exists('Data/'+ DateDir):
    os.makedirs('Data/'+ DateDir)
  
  trialInd = len(os.listdir('Data/'+ DateDir))
  Filename = 'Data/'+ DateDir + "/"+NameofRecord +"_"+ str(trialInd+1) + ".csv"
  
  logFile = open(Filename, 'w', newline = '')
  fileObj = csv.writer(logFile)
  fileObj.writerow(['Sample(20Hz)', 'X_Pos','Y_Pos', 'Z_Pos', 'Roll', 'Pitch', 'Yaw'])



Sample = 0
delta_x = 1 # a big number for initialization
delta_y = 1 # a big number for initialization
delta_z = 1 # a big number for initialization

while np.abs(delta_x) > 0.0001 or np.abs(delta_y) > 0.0001 or np.abs(delta_z) > 0.0001 :
  

  i+=1
  if (useRealTimeSimulation):
    dt = datetime.now()
    t = (dt.second / 60.) * 2. * math.pi
  else:
    t = t + 0.01

  if (useSimulation and useRealTimeSimulation == 0):
    p.stepSimulation()

  delta = time.time() - updateT
  #print(delta) 
  

  if delta > inputRate:
    #print(delta) 
    updateT= time.time()
    # Render and record the image info of our camera with using the same rate
    Image_Info = Camera_Class.render()
    #print(Image_Info)
    
    Sample += 1
    if SaveData:
      lsr = p.getLinkState(jacoId, jacoEndEffectorIndex)
      #lsc = p.getBasePositionAndOrientation(cube1Id)

      #ln = [updateT, lsr[4][0],lsr[4][1],lsr[4][2], lsr[5][0],lsr[5][1], lsr[5][2], lsr[5][3], fing, lsc[0][0],lsc[0][1],lsc[0][2], lsc[1][0],lsc[1][1], lsc[1][2], lsc[1][3]] 
      
      EulerAngles = p.getEulerFromQuaternion(orn)
      ln = [Sample, lsr[4][0],lsr[4][1],lsr[4][2], EulerAngles[0],EulerAngles[1], EulerAngles[2]] 
      ln_rnd = [round(num, 4) for num in ln]
      #print(ln_rnd)
      fileObj.writerow(ln_rnd)

    eulOrn = p.getEulerFromQuaternion(orn)
    Rrm = R.from_quat(orn)

    rx = eulOrn[0]
    ry = eulOrn[1]
    rz = eulOrn[2]

    runUI.update()
    inputMode = runUI.mode
    inputKey  = runUI.state

    baseTheta = JP[0]
    s = math.sin(baseTheta)
    c = math.cos(baseTheta)

    c1 = math.cos(ang)
    s1 = math.sin(ang)

    c2 = math.cos(-ang)
    s2 = math.sin(-ang)

    n = np.sqrt(pos[0]*pos[0] + pos[1]*pos[1])
    dx = -pos[1]/n
    dy = pos[0]/n

    Rnew =  Rrm.as_matrix() 

    if use2D:
      if inputMode == 0:
        if inputKey == 4:
          pos[0] = pos[0] + dist*dx
          pos[1] = pos[1] + dist*dy
          newPosInput = 1
        if inputKey == 6:
          pos[0] = pos[0] - dist*dx
          pos[1] = pos[1] - dist*dy
          newPosInput = 1
        if inputKey == 8:
          pos[0] = pos[0] + dist*c
          pos[1] = pos[1] - dist*s
          newPosInput = 1
        if inputKey == 2:
          pos[0] = pos[0] - dist*c
          pos[1] = pos[1] + dist *s
          newPosInput = 1

      if inputMode ==1:
        if inputKey == 8:
          pos[2] = pos[2] + dist 
          newPosInput = 1
        if inputKey == 2:
          pos[2] = pos[2] - dist 
          newPosInput = 1
        if inputKey == 4:
          Rnew = Rrm.as_matrix() @ Rz
          newPosInput = 1
        if inputKey == 6:
          Rnew = Rrm.as_matrix() @ Rzm
          newPosInput = 1

      if inputMode == 2:
        if inputKey == 8:
          Rnew = Rrm.as_matrix() @ Rx
          newPosInput = 1
        if inputKey == 2:
          Rnew = Rrm.as_matrix() @ Rxm
          newPosInput = 1
        if inputKey == 6:
          Rnew = Rrm.as_matrix() @ Ry
          newPosInput = 1
        if inputKey == 4:
          Rnew = Rrm.as_matrix() @ Rym
          newPosInput = 1

      if inputMode == 3:
        if inputKey == 8:
          fing = fing - dist*5 
        if inputKey == 2:
          fing = fing + dist*5

    else:
      if inputMode == 0:
        if inputKey == 4:
          pos[0] = pos[0] + dist*dx
          pos[1] = pos[1] + dist*dy
          newPosInput = 1
        if inputKey == 6:
          pos[0] = pos[0] - dist*dx
          pos[1] = pos[1] - dist*dy
          newPosInput = 1
        if inputKey == 8:
          pos[0] = pos[0] + dist*c
          pos[1] = pos[1] - dist*s
          newPosInput = 1
        if inputKey == 2:
          pos[0] = pos[0] - dist*c
          pos[1] = pos[1] + dist*s
          newPosInput = 1
        if inputKey == 7:
          pos[2] = pos[2] + dist 
          newPosInput = 1
        if inputKey == 1:
          pos[2] = pos[2] - dist 
          newPosInput = 1
        
      
      if inputMode == 1:
        if inputKey == 4:
          Rnew = Rrm.as_matrix() @ Rz
          newPosInput = 1
        if inputKey == 6:
          Rnew = Rrm.as_matrix() @ Rzm
          newPosInput = 1
        if inputKey == 8:
          Rnew = Rrm.as_matrix() @ Rx
          newPosInput = 1
        if inputKey == 2:
          Rnew = Rrm.as_matrix() @ Rxm
          newPosInput = 1
        if inputKey == 7:
          Rnew = Rrm.as_matrix() @ Ry
          newPosInput = 1
        if inputKey == 1:
          Rnew = Rrm.as_matrix() @ Rym
          newPosInput = 1

      if inputMode == 2:
        if inputKey == 8:
          fing = fing - dist*5 
        if inputKey == 2:
          fing = fing + dist*5

    #Rn = R.from_matrix(Rnew)
    #orn = Rn.as_quat()

    # read positions and orientations of end-effector
    ls = p.getLinkState(jacoId, jacoEndEffectorIndex)
    #print(ls)
    pos = list(ls[4])
    orn = list(ls[5])
    eulOrn = p.getEulerFromQuaternion(orn)
    
    # desgin of a PD controller for set-point control based on error
    newPosInput = 1
    
    delta_x = Obj1_GraspPos[0]-pos[0]
    delta_y = Obj1_GraspPos[1]-pos[1]
    delta_z = Obj1_GraspPos[2]-pos[2]

    #delta_roll = Obj1_GraspOrn[0]-eulOrn[0] 
    #delta_pitch = Obj1_GraspOrn[1]-eulOrn[1]
    #delta_yaw = Obj1_GraspOrn[2]-eulOrn[2] 

    delta_roll = Obj1_GraspOrn[0]-pos[0] 
    delta_pitch = Obj1_GraspOrn[1]-pos[1]
    delta_yaw = Obj1_GraspOrn[2]-pos[2] 



    # gain for controller
    K_p1 = 30 # 30
    k_d1 = 1 # 1
    Gain_x = K_p1*delta_x + k_d1*delta_x/inputRate
    Gain_y = K_p1*delta_y + k_d1*delta_y/inputRate
    Gain_z = K_p1*delta_z + k_d1*delta_z/inputRate 
    
    K_p2 = 10  # 10
    k_d2 = 1 # 1
    Gain_roll = K_p2*delta_roll + k_d2*delta_roll/inputRate
    Gain_pitch = K_p2*delta_pitch + k_d2*delta_pitch/inputRate
    Gain_yaw = K_p2*delta_yaw + k_d2*delta_yaw/inputRate

    # update the end-effector kinematics values
    dv = 0.001 # gain for making small jump
    Gain_x = Gain_x * dv 
    Gain_y = Gain_y * dv
    Gain_z = Gain_z * dv
    Gain_roll = Gain_roll * dv 
    Gain_pitch = Gain_pitch * dv 
    Gain_yaw = Gain_yaw * dv

    pos[0] = pos[0] + Gain_x 
    pos[1] = pos[1] + Gain_y 
    pos[2] = pos[2] + Gain_z 
    #eulOrn = [eulOrn[0] + Gain_roll, eulOrn[1] + Gain_pitch , eulOrn[2] + Gain_yaw]
    eulOrn = [eulOrn[0], eulOrn[1] + Gain_pitch , eulOrn[2]]
    orn = p.getQuaternionFromEuler(eulOrn)
    

    
    '''
    if pos[0] > wu[0]:
      pos[0] =  wu[0]
    if pos[0] < wl[0]:
      pos[0] =  wl[0]
    if pos[1] > wu[1]:
      pos[1] =  wu[1]
    if pos[1] < wl[1]:
      pos[1] =  wl[1]
    if pos[2] > wu[2]:
      pos[2] =  wu[2]
    if pos[2] < wl[2]:
      pos[2] =  wl[2]
    '''
    
    if fing > 1.35:
      fing = 1.35
    if fing < 0:
      fing = 0

  if (newPosInput == 1):
    if (useNullSpace == 1):
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(jacoId, jacoEndEffectorIndex, pos, orn, ll, ul,
                                                  jr, rp)
      else:
        jointPoses = p.calculateInverseKinematics(jacoId,
                                                  jacoEndEffectorIndex,
                                                  pos,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)
    else:
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(jacoId,
                                                  jacoEndEffectorIndex,
                                                  pos,
                                                  orn,
                                                  jointDamping=jd,
                                                  solver=ikSolver,
                                                  maxNumIterations=100,
                                                  residualThreshold=.01)
        JP = list(jointPoses)
        
      else:
        jointPoses = p.calculateInverseKinematics(jacoId,
                                                  jacoEndEffectorIndex,
                                                  pos,
                                                  solver=ikSolver)
        JP = list(jointPoses)

  if (useSimulation):
    
    JS = p.getJointStates(jacoId, [1, 2, 3, 4, 5, 6, 7, 9, 11, 13])
    j = 0
    for i in [2,3,4,5,6,7]:
      p.setJointMotorControl2(jacoId, i, p.POSITION_CONTROL, JP[j])
      j = j+1
    
    for i in  [9, 11, 13]:
      p.setJointMotorControl2(jacoId, i, p.POSITION_CONTROL, fing)
    
    # after one-step move, should render again
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
      

  else:
    j = 0
    for i in jacoArmJoints:
      p.resetJointState(jacoId, i, jointPoses[j])
      j = j+1

  ls = p.getLinkState(jacoId, jacoEndEffectorIndex)
  #print(ls)
  # positions and orientations of end-effector
  pos = list(ls[4])
  orn = list(ls[5])

  newPosInput = 0


file.close()
p.disconnect()

# save the kinematics data of the end-effector
# save to csv files
#savetxt('~/Repositories/KinovaPyBullet/Data/End_Effector_Pos.csv', End_Effector_Pos, delimiter=',')
#savetxt('~/Repositories/KinovaPyBullet/Data/End_Effector_Orn.csv', End_Effector_Orn, delimiter=',')
#!/usr/bin/env python
# coding: utf-8

# In[1]:


import pybullet as p
import time
import math

class Simulation(object):

    def __init__(self):

        self.id=p.connect(p.GUI)

        p.resetSimulation()

        p.setGravity(0,0,-10)
        useRealTimeSim = 0

        p.setTimeStep(1./120.)
        p.setRealTimeSimulation(useRealTimeSim) # either this

        track = p.loadURDF("plane.urdf")

        self.objects=[]

    def disconnect(self):
        p.disconnect(self.id)

    def update(self):
        for obj in self.objects:
            obj.update()
        p.stepSimulation()



class Car(object):

    def __init__(self,initial_position=[0,0,0.3]):

        self.car=p.loadURDF("f10_racecar/racecar_differential.urdf",
                        initial_position)

        self.action_taken=True
        self.wheel_velocity_value=0.0
        self.max_force_value=50.0
        self.steering_value=0.0


        car=self.car
        for wheel in range(p.getNumJoints(car)):
            print("joint[",wheel,"]=", p.getJointInfo(car,wheel))
            p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
            p.getJointInfo(self.car,wheel)    

            c = p.createConstraint(car,9,car,11,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
            p.changeConstraint(c,gearRatio=1, maxForce=10000)

            c = p.createConstraint(car,10,car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
            p.changeConstraint(c,gearRatio=-1, maxForce=10000)

            c = p.createConstraint(car,9,car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
            p.changeConstraint(c,gearRatio=-1, maxForce=10000)

            c = p.createConstraint(car,16,car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
            p.changeConstraint(c,gearRatio=1, maxForce=10000)


            c = p.createConstraint(car,16,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
            p.changeConstraint(c,gearRatio=-1, maxForce=10000)

            c = p.createConstraint(car,17,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
            p.changeConstraint(c,gearRatio=-1, maxForce=10000)

            c = p.createConstraint(car,1,car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
            p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15, maxForce=10000)
            c = p.createConstraint(car,3,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
            p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15,maxForce=10000)

        self.update()


    @property
    def yaw(self):
        import math
        carPos,carOrn = p.getBasePositionAndOrientation(self.car)
        carEuler = p.getEulerFromQuaternion(carOrn)
        carYaw = carEuler[2]*360/(2.*math.pi)-90
        return carYaw
        
    @property
    def steering(self):
        return self.steering_value

    @steering.setter
    def steering(self,val):
        self.steering_value=val
        self.action_taken=True

    @property
    def max_force(self):
        return self.max_force_value

    @max_force.setter
    def max_force(self,val):
        self.max_force_value=val
        self.action_taken=True


    @property
    def wheel_velocity(self):
        return self.wheel_velocity_value

    @wheel_velocity.setter
    def wheel_velocity(self,val):
        if val<-50:
            val=-50

        if val>50:
            val=50

        if val==self.wheel_velocity_value:  # no change
            return

        self.wheel_velocity_value=val
        self.action_taken=True

    def update(self):
        if not self.action_taken:
            return

        steering = [0,2]  # ids from the model for the steering mechanism
        wheels = [8,15]

        print(self.wheel_velocity,self.max_force_value)
        for wheel in wheels:
            p.setJointMotorControl2(self.car,wheel,p.VELOCITY_CONTROL,targetVelocity=self.wheel_velocity,
            force=self.max_force_value)
            
        self.action_taken=False

        for steer in steering:
            p.setJointMotorControl2(self.car,steer,p.POSITION_CONTROL,targetPosition=-self.steering)
            


class CameraView(object):

    def __init__(self,car):
        self.car=car
        self.camInfo = p.getDebugVisualizerCamera()
        self.viewtime=-100

    def update(self):
        nowTime = time.time()
        lastTime=self.viewtime
        car=self.car.car
        zed_camera_joint = 5

        if (nowTime-lastTime>.1):
            ls = p.getLinkState(car,zed_camera_joint, computeForwardKinematics=True)
            camPos = ls[0]
            camOrn = ls[1]
            camMat = p.getMatrixFromQuaternion(camOrn)
            upVector = [0,0,1]
            forwardVec = [camMat[0],camMat[3],camMat[6]]
            #sideVec =  [camMat[1],camMat[4],camMat[7]]
            camUpVec =  [camMat[2],camMat[5],camMat[8]]
            camTarget = [camPos[0]+forwardVec[0]*10,camPos[1]+forwardVec[1]*10,camPos[2]+forwardVec[2]*10]
            camUpTarget = [camPos[0]+camUpVec[0],camPos[1]+camUpVec[1],camPos[2]+camUpVec[2]]
            viewMat = p.computeViewMatrix(camPos, camTarget, camUpVec)
            projMat = self.camInfo[3]
            #p.getCameraImage(320,200,viewMatrix=viewMat,projectionMatrix=projMat, flags=p.ER_NO_SEGMENTATION_MASK, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            p.getCameraImage(320,200,viewMatrix=viewMat,projectionMatrix=projMat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            self.viewtime=lastTime=nowTime

import time
class Timer(object):

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_time=time.time()

    def __call__(self):
        return time.time()-self.last_time

    def __repr__(self):
        from datetime import timedelta
        dt=timedelta(seconds=self())
        return str(dt)

import threading
def run_car(car):

    print("Stopping Car.")

    car.wheel_velocity=50
    time.sleep(2)

    car.wheel_velocity=-50
    car.steering=2
    time.sleep(2)


    car.steering=0
    car.wheel_velocity=50
    time.sleep(2)


    car.wheel_velocity=0
    time.sleep(2)


    print("Stopping Car.")


sim=Simulation()

otherCar=Car([0,1,.3])
car=Car([0,0,.3])
view=CameraView(car)

sim.objects=[car,otherCar,view]

sim.update()

thread = threading.Thread(target=run_car, args=(car,))
thread.start()

tm=Timer()

try:
    while (True):
        sim.update()
        time.sleep(0.01)

        if not thread.isAlive():
            break

except KeyboardInterrupt:
    pass

sim.disconnect()


# In[2]:


sim.disconnect()


# In[2]:


del sim


# In[4]:


del car 


# In[5]:


del otherCar


# In[6]:


del view


# In[7]:


dir()


# In[8]:


del In
del Out


# In[9]:


thread.join()


# In[10]:


del thread


# In[11]:


del Simulation


# In[12]:


del CameraView


# In[13]:


del Car


# In[14]:


dir


# In[15]:


dir()


# In[21]:


del p


# In[22]:


del run_car


# In[23]:


del threading


# In[ ]:





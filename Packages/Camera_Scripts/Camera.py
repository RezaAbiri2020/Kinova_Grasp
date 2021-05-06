
import pybullet as p
import numpy as np
import os
import time 

# consider a camera; record the images; analysis the image to capture necessary 6D object pose or 6D grasp pose 

class IntelCamera():
    
    def __init__(self, videoname, record):

        # positioning the camera 

        #p.resetDebugVisualizerCamera(cameraDistance=0.20, cameraYaw=10, cameraPitch=-30, cameraTargetPosition=[-0.4,-0.35,0.0])
        
        # to look from the top use the following:
        p.resetDebugVisualizerCamera(cameraDistance=1.70, cameraYaw=30, cameraPitch=-90, cameraTargetPosition=[-0.64,0.0,0.0])

        #p.resetDebugVisualizerCamera(cameraDistance=1.7, cameraYaw=40, cameraPitch=-45, cameraTargetPosition=[-0.64,0.0,0.0])
        
        # this is to set the OpenGL active (1; default internal value) to show the synthetic cameras or not showing them (0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        
        self.videoname = videoname
        self.record = record


    def video(self):
        # to record a video from this camera uncomment this line
        if self.record:
            if not os.path.exists('Videos'):
                os.makedirs('Videos')
            DateDir = time.strftime("%Y%m%d")
            if not os.path.exists('Videos/'+ DateDir):
                os.makedirs('Videos/'+ DateDir)
            
            Count = len(os.listdir('Videos/'+ DateDir))        
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./Videos/"+DateDir+"/"+self.videoname+"_"+str(Count+1)+".MP4")
        else:
            pass

   
    def render(self):
        
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[-0.45, -0.45, 0.2],
                                                            distance=1.5,
                                                            yaw=90,
                                                            pitch=-90,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (width, height, rgbPixels, depthPixels, SegMaskBuffer) = p.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(rgbPixels, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (height, width, 4))
        
        #depthPixels = np.
        
        # the default internal values are (1; enabled); if 0 will be disabled.
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)

        # return a tuple for rgb data and depth values
        return (rgb_array, depthPixels)










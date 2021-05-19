#! /usr/bin/env python
import rospy
import numpy as np
import time
import cv2 
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_quat
bridge=CvBridge()
sift = cv2.xfeatures2d.SIFT_create()

#Function for finding initial pose matrix
def callback_init(pm_ini):
    init_t=np.array([[pm_ini.pose.pose.position.x],[pm_ini.pose.pose.position.y],[pm_ini.pose.pose.position.z]])
    j=R_quat.from_quat([pm_ini.pose.pose.orientation.x,pm_ini.pose.pose.orientation.y,pm_ini.pose.pose.orientation.z,pm_ini.pose.pose.orientation.w])
    init_rot=j.as_dcm()
    init_final=np.concatenate((np.concatenate((init_rot,init_t),axis=-1),np.array([[0,0,0,1]])),axis=0)
    return init_final

#Function for finding Ground Truth Matrix
def callback_gt(gt_msg):
    global gt_x,gt_y,gt_z
    gt_x.append(gt_msg.pose.pose.position.x)
    gt_y.append(gt_msg.pose.pose.position.y)
    gt_z.append(gt_msg.pose.pose.position.z)


def callback_matching(image_message):
    global i ,des2 ,kp_img2,img2,k,R,t,trans,rot,final_c2b,store,plots 

    img1= bridge.imgmsg_to_cv2(image_message,"bgr8")
    gray_img1= cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)

    #Feature Detection
    kp_img1,des1=sift.detectAndCompute(gray_img1,None)
    if i==0:
        pass
    else:
        #Feature Matching
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1,des2,k=2)

        # Apply ratio test
        bad = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                bad.append([m])

        # cv.drawMatchesKnn expects list of lists as matches.
        img3 = cv2.drawMatchesKnn(img1,kp_img1,img2,kp_img2,bad,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow('image', img3)
        cv2.waitKey(1)
        good = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append(m)        
        src_p = np.float32([ kp_img1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_p=np.float32([kp_img2[m.trainIdx].pt for m in good]).reshape(-1,1,2)
    
        pts1 = np.float32(src_p)
        pts2 = np.float32(dst_p)

        R = []
        t = []    

        #Find Fundamental Matrix
        F,mask = cv2.findFundamentalMat(pts1, pts2,cv2.FM_RANSAC ,0.4,0.9,mask=None)

        #Find Essential Matrix
        E = np.matmul(k.T,(np.matmul(F,k)))

        #Find rotation and translation between two consecutive frames
        points,R, t, mask = cv2.recoverPose(E, pts1, pts2, k)
        
        #Find the rotation and translation wrt first frame
        if i == 1: 
            m=np.concatenate((np.concatenate((R,np.matmul(R,t)),axis=-1),np.array([[0,0,0,1]])),axis=0)
            store[i-1]=np.matmul(pose_mat,m)
            plots[i-1]=np.matmul(final_c2b,store[i-1])

        if i>1:
            f1_r=np.matmul(store[i-2][:3,:3],R)
            f2_t=store[i-2][:3,3].reshape(3,1) - np.matmul(f1_r,t)
            store[i-1]=np.concatenate((np.concatenate((f1_r,f2_t),axis=-1),np.array([[0,0,0,1]])),axis=0)
            plots[i-1]=np.matmul(final_c2b,store[i-1])

    img2=img1
    kp_img2=kp_img1
    des2=des1
    i=i+1
   
def main():
    print("sania1")
    global i,k,plots,gt_x,gt_y,gt_z,final_c2b,pose_mat,store
    store={}
    gt_x=[]
    gt_y=[]
    gt_z=[]
    plots={}
   
    i=0
    rospy.init_node('vodom_sub', anonymous=True)

    #camera calibration matrix
    first_message = rospy.wait_for_message('/r200/camera/color/camera_info',CameraInfo,timeout=None)
    k=np.asarray(first_message.K).reshape((3, 3))

    #Find initial Pose
    init_pose=rospy.wait_for_message('/ground_truth/state',Odometry,timeout=None)
    pose_mat=callback_init(init_pose)

    #Find camera to base link transformation matrix
    b2c_msg=_=rospy.wait_for_message('/tf_static',TFMessage,timeout=None)
    h=b2c_msg.transforms[5].transform
    #translation 
    tran_b2c=np.array([[h.translation.x],[h.translation.y],[h.translation.z]])    
    #rotation
    u=R_quat.from_quat([h.rotation.x,h.rotation.y,h.rotation.z,h.rotation.w ])
    rot_b2c=u.as_dcm()
    #final matrix 
    p=np.concatenate((np.concatenate((rot_b2c,tran_b2c),axis=-1),np.array([[0,0,0,1]])),axis=0)
    print('p is',p,p.shape)
    final_c2b=np.linalg.inv(p)
    
    cam_subscriber = rospy.Subscriber('/r200/camera/color/image_raw',Image,callback_matching)

    #Ground truth
    ground_truth=rospy.Subscriber('/ground_truth/state',Odometry,callback_gt)
    
    rospy.spin()


if __name__ == '__main__':

    try:
        print("Sania0")
        main()
        print("end")
        #Data for plotting estimated trajectory
        x0=[]
        y0=[]
        for a in plots:
            y0.append(plots[a][1,3])
            x0.append(plots[a][0,3])

        #Data for plotting ground truth
        x2 = gt_x
        y2 = gt_y
         
        plt.plot(x0, y0)
        plt.plot(x2, y2)
        plt.legend([[x0,y0],[x2,y2]])
        plt.show()
  
    except rospy.ROSInterruptException: pass
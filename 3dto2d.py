#! /usr/bin/env python
import rospy
import numpy as np
import time
import cv2 
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.spatial.transform import Rotation as R_quat
from sensor_msgs import point_cloud2
matplotlib.use('TkAgg')
from mpl_toolkits.mplot3d import Axes3D
#plt.ion()
bridge = CvBridge()
orb=cv2.ORB_create()
index_params = dict(
    algorithm=6, table_number=6, key_size=12, multi_probe_level=1  # 12  # 20
)  # 2
search_params = dict(checks=100)
flann = cv2.FlannBasedMatcher(index_params, search_params)
global prev_kp,prev_des,curr_kp,curr_des,prev_cloud,final_pose,trajectory,gt_x,gt_y
prev_des=[]
final_pose=[]
trajectory=[]
gt_x=[]
gt_y=[]


def callback_gt(gt_msg):
    global gt_x,gt_y,gt_z
    gt_x.append(gt_msg.pose.pose.position.x)
    gt_y.append(gt_msg.pose.pose.position.y)
    #gt_z.append(gt_msg.pose.pose.position.z)


def ratio_test(flann_match):
    good=[]
    for match in flann_match:
        if len(match) == 2:
            m, n = match
            if m.distance < 0.75 * n.distance:
                good.append([m,n])
    return good


def corresponding_features(g_match,kp1,kp2,c1,c2):
    xyz = []
    xy = []
    for m,n in g_match:
        keypoint=kp1[m.queryIdx]
        x2,y2=kp2[m.trainIdx].pt
        point=np.asarray(c1[int(keypoint.pt[1]),int(keypoint.pt[0])])
        #print("point from cloud",point)
        x2, y2 = kp2[m.trainIdx].pt
        if not np.isnan(point).any():
            xyz.append(point)
            xy.append([x2,y2])
    return np.asarray(xyz),np.asarray(xy)


def callback_func(img_msg,cloud_msg):
    global prev_kp,prev_des,curr_kp,curr_des,curr_cloud,prev_cloud,k,final_pose,trajectory
    #FIND KEYPOINTS OF THE CURRENT IMAGE
    curr_img= bridge.imgmsg_to_cv2(img_msg, "bgr8")
    curr_kp,curr_des=orb.detectAndCompute(curr_img,None)

    #MATCH THE FEATURES OF THE CURRENT IMAGE AND PREVIOUS IMAGE
    matches = flann.knnMatch(prev_des,curr_des, k=2)
    
    #CURRENT POINT CLOUD
    curr_cloud = np.asarray(list(
        point_cloud2.read_points(
            cloud_msg, field_names=("x", "y", "z"), skip_nans=False
        ))).reshape(curr_img.shape)

    #FINDING MATCHES THAT SATISFY THE RATIO TEST
    good_matches=ratio_test(matches)


    #FIND CORRESPONDING POINTS OF THE THE GOOD MATCHES
    if good_matches is not None:
        xyz_coords,xy_img_coords=corresponding_features(good_matches, prev_kp, curr_kp, prev_cloud, curr_cloud)

    #OBTAIN ROTATION AND TRANSLATION VECTORS BETWEEN CONSECUTIVE FRAMES USING PnP    
    _, rot_vec, trans_vec, _ = cv2.solvePnPRansac(xyz_coords, xy_img_coords, k, None)  

    #CONVERT ROTATION VECTOR TO ROTATION MATRIX 
    rot_mat, _ = cv2.Rodrigues(rot_vec)


    curr_tranform=np.concatenate((np.concatenate((rot_mat,trans_vec),axis=-1),np.array([[0,0,0,1]])),axis=0)


    final_pose.append(np.matmul(final_pose[-1],curr_tranform))
    #gt_x.append(gt_msg.pose.pose.position.x)
    #gt_y.append(gt_msg.pose.pose.position.y)
    #gt_z.append(gt_msg.pose.pose.position.z)   
    
    prev_des=curr_des.copy()
    prev_kp=curr_kp
    prev_cloud=curr_cloud.copy()



def main():
    global prev_kp,prev_des,curr_kp,curr_des,prev_cloud,final_pose,k,final_c2b
    rospy.init_node('optical_pnp',anonymous=True)
    #CAMERA CALLIBRATION MATRIX
    first_message = rospy.wait_for_message('/r200/camera/color/camera_info',CameraInfo,timeout=None)
    k=np.asarray(first_message.K).reshape((3, 3))

    #FIRST IMAGE
    prev_img= rospy.wait_for_message("/r200/camera/color/image_raw", Image)
    prev_img= bridge.imgmsg_to_cv2(prev_img, "bgr8")
    #FINDING KEPOINTS OF FIRST FRAME
    prev_kp,prev_des=orb.detectAndCompute(prev_img,None)
    print("length in main",len(prev_kp))

    #FIRST POINT CLOUD
    prev_pc=rospy.wait_for_message('/camera/depth/points', PointCloud2)
    prev_cloud = np.asarray(list(
        point_cloud2.read_points(
            prev_pc, field_names=("x", "y", "z"), skip_nans=False
        ))).reshape(prev_img.shape)

    #FIND INITIAL POSE 
    pm_ini=rospy.wait_for_message('/ground_truth/state',Odometry,timeout=None)

    #print("pm_init",pm_ini)
    init_t=np.array([[pm_ini.pose.pose.position.x],[pm_ini.pose.pose.position.y],[pm_ini.pose.pose.position.z]])
    j=R_quat.from_quat([pm_ini.pose.pose.orientation.x,pm_ini.pose.pose.orientation.y,pm_ini.pose.pose.orientation.z,pm_ini.pose.pose.orientation.w])
    init_rot=j.as_dcm()
    init_pose=np.concatenate((np.concatenate((init_rot,init_t),axis=-1),np.array([[0,0,0,1]])),axis=0)    
    final_pose.append(init_pose)

    img_subscriber = Subscriber('/r200/camera/color/image_raw',Image)
    pc_subscriber=Subscriber('/camera/depth/points',PointCloud2)
    ats = ApproximateTimeSynchronizer([img_subscriber, pc_subscriber], 100, 100, allow_headerless=True)
    ats.registerCallback(callback_func)
    ground_truth=rospy.Subscriber('/ground_truth/state',Odometry,callback_gt)
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
        c=np.asarray(final_pose)
        plt.plot(list(c[:,0,3]),list(-c[:,1,3]))
        plt.plot(gt_x,gt_y)
        plt.legend([['trajectory'],['ground truth']])
        plt.show()
    except rospy.ROSInterruptException: pass




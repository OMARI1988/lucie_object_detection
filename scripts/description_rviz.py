#!/usr/bin/env python

import rospy
import copy
import numpy as np

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
import glob
from tf.transformations import euler_from_quaternion
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class people():
    """docstring for people."""
    def __init__(self):
        # self.menu_handler = MenuHandler()
        self._get_faces()
        self.person = {}
        self.frame = 0
        self.counter = 0
        self.counter2 = 0
        self.video = self.ok_videos[0]
        self.how_many_frames = 10
        self._get_next_video()
        self.data = {}      # use this later to make it online
        self.robot = {}
        self.colour = {}
        self.bridge = CvBridge()
        self.action = ""
        self.image_pub = rospy.Publisher("camera_replay", Image, queue_size=2)

    def _get_faces(self):
        self.ok_videos = []
        self.dir = "/home/omari/Datasets/ECAI_dataset/faces/faces3/"
        ok_videos = [x[0] for x in os.walk(self.dir)]
        ok_videos = sorted(ok_videos[1:])
        for i in ok_videos:
            self.ok_videos.append(int(i.split("_")[-1]))
        # self.ok_videos = [106, 119, 108] 


    def _get_next_video(self):
        print self.video
        self._skl_dir = "/home/omari/Datasets/ECAI Data/dataset_segmented_15_12_16/vid"+str(self.video)+"/skeleton/"
        self._skl_files = sorted(glob.glob(self._skl_dir+"*.txt"))
        self._rbt_dir = "/home/omari/Datasets/ECAI Data/dataset_segmented_15_12_16/vid"+str(self.video)+"/robot/"
        self._rbt_files = sorted(glob.glob(self._rbt_dir+"*.txt"))
        self._img_dir = "/home/omari/Datasets/ECAI Data/dataset_segmented_15_12_16/vid"+str(self.video)+"/images/"
        self._img_files = sorted(glob.glob(self._img_dir+"*.jpg"))
        self._clr_dir = "/home/omari/Datasets/ECAI_dataset/features/vid"
        self.frames = [0]
        for i in range(len(self._skl_files)):
            self.frames.append( i )
        self.frames.append(-1)
        self.counter2 = 0
        self.frame = 0
        print '-----------'

    def frameCallback( self, msg ):
        self.get_sk_info()
        self._get_color_info()
        self._get_action_info()
        self._pub_image()
        for name in self.person:
            #UPDATE BODY
            self.person[name]["lower"].scale = (self.data["head"][2]-.25)/2.0
            self.person[name]["lower"].pose.position.x = self.data["torso"][0]
            self.person[name]["lower"].pose.position.y = self.data["torso"][1]
            self.person[name]["lower"].pose.position.z = (self.data["head"][2]-.25)/4.0
            self.person[name]["lower"].controls[0].markers[0].scale.z = (self.data["head"][2]-.25)/2.0
            self.person[name]["lower"].controls[0].markers[0].color.r = self.colour["lower"][2]
            self.person[name]["lower"].controls[0].markers[0].color.g = self.colour["lower"][1]
            self.person[name]["lower"].controls[0].markers[0].color.b = self.colour["lower"][0]

            self.person[name]["upper"].scale = (self.data["head"][2]-.25)/2.0
            self.person[name]["upper"].pose.position.x = self.data["torso"][0]
            self.person[name]["upper"].pose.position.y = self.data["torso"][1]
            self.person[name]["upper"].pose.position.z = 3*(self.data["head"][2]-.25)/4.0
            self.person[name]["upper"].controls[0].markers[0].scale.z = (self.data["head"][2]-.25)/2.0
            self.person[name]["upper"].controls[0].markers[0].color.r = self.colour["upper"][2]
            self.person[name]["upper"].controls[0].markers[0].color.g = self.colour["upper"][1]
            self.person[name]["upper"].controls[0].markers[0].color.b = self.colour["upper"][0]

            self.person[name]["head"].pose.position.x = self.data["torso"][0]
            self.person[name]["head"].pose.position.y = self.data["torso"][1]
            self.person[name]["head"].pose.position.z = self.data["head"][2]
            # self.action.replace("\n",",")
            self.person[name]["head"].description = "name: unknown"#self.action
            self.person[name]["head"].description += "\n"#self.action
            self.person[name]["head"].description += "upper: _"#self.action
            self.person[name]["head"].description += "\n"#self.action
            self.person[name]["head"].description += "lower: _"#self.action
            self.person[name]["head"].description += "\n"#self.action
            self.person[name]["head"].description += "action: _"#self.action

            self.person[name]["right_hand"].pose.position.x = self.data["right_hand"][0]
            self.person[name]["right_hand"].pose.position.y = self.data["right_hand"][1]
            self.person[name]["right_hand"].pose.position.z = self.data["right_hand"][2]

            self.person[name]["left_hand"].pose.position.x = self.data["left_hand"][0]
            self.person[name]["left_hand"].pose.position.y = self.data["left_hand"][1]
            self.person[name]["left_hand"].pose.position.z = self.data["left_hand"][2]

            for part in self.person[name]:
                self.server.insert(self.person[name][part], self.processFeedback)
        self.server.applyChanges()
        self.frame = self.frames[self.counter2]
        self.counter2+=1
        if self.frame == -1:
            if self.counter == len(self.ok_videos):
                self.video = self.ok_videos[0]
                self.counter = 0
            else:
                self.counter += 1
                self.video = self.ok_videos[self.counter]
            self.frame = 0
            self._get_next_video()

    def _get_color_info(self):
        f1 = open(self._clr_dir+str(self.video)+"/colours.txt",'r')
        for count, line in enumerate(f1):
            if count == self.frame:
                C = map(int, line.split('\n')[0].split(","))
                self.colour["upper"] = [x / 255.0 for x in C]
            if count == len(self._skl_files)+2+self.frame:
                C = map(int, line.split('\n')[0].split(","))
                self.colour["lower"] = [x / 255.0 for x in C]

    def _get_action_info(self):
        f1 = open(self._clr_dir+str(self.video)+"/actions.txt",'r')
        for count, line in enumerate(f1):
            self.action = line

    def _pub_image(self):
        img = cv2.imread(self._img_files[self.frame])
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def _convert_to_world_frame(self, xyz):
        """Convert a single camera frame coordinate into a map frame coordinate"""
        fx = 525.0
        fy = 525.0
        cx = 319.5
        cy = 239.5

        y,z,x = xyz[0], xyz[1], xyz[2]

        xr = self.robot['x']
        yr = self.robot['y']
        zr = self.robot['z']
        roll = self.robot['rol']
        pr   = self.robot['pit']
        yawr = self.robot['yaw']

        # # yawr += robot_msg.PTU_pan
        # # pr += robot_msg.PTU_tilt
        #
        # transformation from camera to map
        rot_y = np.matrix([[np.cos(pr), 0, np.sin(pr)], [0, 1, 0], [-np.sin(pr), 0, np.cos(pr)]])
        rot_z = np.matrix([[np.cos(yawr), -np.sin(yawr), 0], [np.sin(yawr), np.cos(yawr), 0], [0, 0, 1]])
        rot = rot_z*rot_y

        pos_r = np.matrix([[xr], [yr], [zr+1.66]]) # robot's position in map frame
        pos_p = np.matrix([[x], [-y], [z]]) # person's position in camera frame

        map_pos = rot*pos_p+pos_r # person's position in map frame
        x_mf = map_pos[0,0]
        y_mf = map_pos[1,0]
        z_mf = map_pos[2,0]

        return [x_mf, y_mf, z_mf]

    def _get_robot_msg(self):
        f1 = open(self._rbt_files[self.frame],'r')
        for count, line in enumerate(f1):
            # read the x value
            if count == 1:
                a = float(line.split('\n')[0].split(':')[1])
                self.robot['x'] = a
            # read the y value
            elif count == 2:
                a = float(line.split('\n')[0].split(':')[1])
                self.robot['y'] = a
            # read the z value
            elif count == 3:
                a = float(line.split('\n')[0].split(':')[1])
                self.robot['z'] = a
            # read roll pitch yaw
            elif count == 5:
                ax = float(line.split('\n')[0].split(':')[1])
            elif count == 6:
                ay = float(line.split('\n')[0].split(':')[1])
            elif count == 7:
                az = float(line.split('\n')[0].split(':')[1])
            elif count == 8:
                aw = float(line.split('\n')[0].split(':')[1])
                # ax,ay,az,aw
                roll, pitch, yaw = euler_from_quaternion([ax, ay, az, aw])    #odom
                pitch = 10*math.pi / 180.   #we pointed the pan tilt 10 degrees
                self.robot['rol'] = roll
                self.robot['pit'] = pitch
                self.robot['yaw'] = yaw

    def get_sk_info(self):
        self._get_robot_msg()
        f1 = open(self._skl_files[self.frame],'r')
        joints = {}
        for count, line in enumerate(f1):
            if count == 0:
                t = np.float64(line.split(':')[1].split('\n')[0])
            # read the joint name
            elif (count-1)%10 == 0:
                j = line.split('\n')[0]
                joints[j] = []
            # read the x value
            elif (count-1)%10 == 2:
                a = float(line.split('\n')[0].split(':')[1])
                joints[j].append(a)
            # read the y value
            elif (count-1)%10 == 3:
                a = float(line.split('\n')[0].split(':')[1])
                joints[j].append(a)
            # read the z value
            elif (count-1)%10 == 4:
                a = float(line.split('\n')[0].split(':')[1])
                joints[j].append(a)
                self.data[j] = self._convert_to_world_frame(joints[j])


    # def get_openni_values(self):
    #     robot = self._get_robot_msg()
    #     f1 = open(self._skl_files[self.frame],'r')
    #     self.data = {}
    #     for count, line in enumerate(f1):
    #         if count == 0: continue
    #         line = line.split(',')
    #         joint_name = line[0]
    #         self.data[joint_name] = {}
    #         x2d = float(line[1])
    #         y2d = float(line[2])
    #         x = float(line[3])
    #         y = float(line[4])
    #         z = float(line[5])
    #         # self._convert_to_world_frame([x,y,z],[rx,ry,rz])
    #         self.data[joint_name] = [x,y,z]
    #     self.frame+=1
    #     if self.frame == len(self._skl_files):
    #         self.frame=0

    def processFeedback( self, feedback ):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo( s + ": button click" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( s + ": pose changed")

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo( s + ": mouse down" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )
        server.applyChanges()

    def makeCyl( self, msg, rgb ):
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = msg.scale
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        return marker

    def makeCir( self, msg, rgb ):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale
        marker.scale.y = msg.scale
        marker.scale.z = msg.scale
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        return marker

    def makeBox( self, msg, rgb ):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = msg.scale
        marker.scale.y = msg.scale
        marker.scale.z = msg.scale
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        return marker

    def makeCirControl( self, msg, rgb ):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeCir(msg,rgb) )
        msg.controls.append( control )
        return control

    def makeBoxControl( self, msg, rgb ):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeBox(msg,rgb) )
        msg.controls.append( control )
        return control

    def makeCylControl( self, msg, rgb ):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeCyl(msg,rgb) )
        msg.controls.append( control )
        return control

    def _make_lower(self,p,name):
        # make lower
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose.position = Point(p[0],p[1],(p[2]-.25)/4.0)
        int_marker.scale = (p[2]-.25)/2.0
        int_marker.name = "person_"+name+"_lower"
        color = [0,0,.5]
        self.makeCylControl(int_marker,color)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
        return int_marker

    def _make_upper(self,p,name):
        # make upper
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose.position = Point(p[0],p[1],3*(p[2]-.25)/4.0)
        int_marker.scale = (p[2]-.25)/2.0
        int_marker.name = "person_"+name+"_upper"
        color = [.5,0,0]
        self.makeCylControl(int_marker,color)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
        return int_marker

    def _make_head(self,p,name):
        # make upper
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose.position = Point(p[0],p[1],3*(p[2]-.25)/4.0)
        int_marker.scale = .45
        int_marker.name = "person_"+name+"_head"
        int_marker.description = "person_1"
        color = [.7,.7,.7]
        self.makeCirControl(int_marker,color)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
        return int_marker

    def _make_RH(self,p,name):
        # make right hand
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose.position = Point(p[0],p[1],p[2])
        int_marker.scale = 0.15
        int_marker.name = "person_"+name+"_right_hand"
        color = [1,.3,.3]
        self.makeBoxControl(int_marker,color)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
        return int_marker

    def _make_LH(self,p,name):
        # make right hand
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose.position = Point(p[0],p[1],p[2])
        int_marker.scale = 0.15
        int_marker.name = "person_"+name+"_left_hand"
        color = [1,.3,.3]
        self.makeBoxControl(int_marker,color)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
        return int_marker

    #####################################################################
    # People Creation
    def _create_person(self, p, name):
        self.person[name] = {}
        self.person[name]['lower'] = self._make_lower(p,name)
        self.person[name]['upper'] = self._make_upper(p,name)
        self.person[name]['head'] = self._make_head(p,name)
        self.person[name]['right_hand'] = self._make_RH(p,name)
        self.person[name]['left_hand'] = self._make_LH(p,name)

        # # insert it
        for part in self.person[name]:
            self.server.insert(self.person[name][part], self.processFeedback)
            # self.menu_handler.apply( self.server, self.person[name][part].name )


if __name__=="__main__":
    rospy.init_node("basic_controls")
    # br = TransformBroadcaster()

    # create a timer to update the published transforms
    P = people()
    P.server = InteractiveMarkerServer("basic_controls")
    rospy.Timer(rospy.Duration(0.02), P.frameCallback)

    # P.menu_handler.insert( "First Entry", callback=processFeedback )
    # P.menu_handler.insert( "Second Entry", callback=processFeedback )
    # P.sub_menu_handle = P.menu_handler.insert( "Submenu" )
    # P.menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    # P.menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )

    position = [-3, 3, 1.8]
    P._create_person( position, '1')

    P.server.applyChanges()
    # while not rospy.is_shutdown:
    #     pass
    rospy.spin()

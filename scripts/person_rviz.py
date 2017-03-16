#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
import glob


class people():
    """docstring for people."""
    def __init__(self):
        # self.menu_handler = MenuHandler()
        self.person = {}
        self.tt = 1
        self._data_dir = "/home/omari/Datasets/Lucie/2017-03-10/2017-03-10_10:14:56_a18c6ce9-f771-5eb1-875b-297b7f03280d/skeleton/"
        self._data_files = sorted(glob.glob(self._data_dir+"*.txt"))
        self.frame = 0
        self.data = {}      # use this later to make it online

    def frameCallback( self, msg ):
        self.get_openni_values()
        for name in self.person:
            #UPDATE BODY
            self.person[name]["lower"].scale = (self.data["head"][2]-.25)/2.0
            self.person[name]["lower"].pose.position.x = self.data["torso"][0]
            self.person[name]["lower"].pose.position.y = self.data["torso"][1]
            self.person[name]["lower"].pose.position.z = (self.data["head"][2]-.25)/4.0
            self.person[name]["lower"].controls[0].markers[0].scale.z = (self.data["head"][2]-.25)/2.0

            self.person[name]["upper"].scale = (self.data["head"][2]-.25)/2.0
            self.person[name]["upper"].pose.position.x = self.data["torso"][0]
            self.person[name]["upper"].pose.position.y = self.data["torso"][1]
            self.person[name]["upper"].pose.position.z = 3*(self.data["head"][2]-.25)/4.0
            self.person[name]["upper"].controls[0].markers[0].scale.z = (self.data["head"][2]-.25)/2.0

            self.person[name]["head"].pose.position.x = self.data["torso"][0]
            self.person[name]["head"].pose.position.y = self.data["torso"][1]
            self.person[name]["head"].pose.position.z = self.data["head"][2]
            print self.data["right_hand"]
            self.person[name]["right_hand"].pose.position.x = self.data["right_hand"][0]
            self.person[name]["right_hand"].pose.position.y = self.data["right_hand"][1]
            self.person[name]["right_hand"].pose.position.z = self.data["right_hand"][2]

            self.person[name]["left_hand"].pose.position.x = self.data["left_hand"][0]
            self.person[name]["left_hand"].pose.position.y = self.data["left_hand"][1]
            self.person[name]["left_hand"].pose.position.z = self.data["left_hand"][2]

            for part in self.person[name]:
                self.server.insert(self.person[name][part], self.processFeedback)
        self.server.applyChanges()

    def get_openni_values(self):
        f1 = open(self._data_files[self.frame],'r')
        self.data = {}
        for count, line in enumerate(f1):
            if count == 0: continue
            line = line.split(',')
            joint_name = line[0]
            self.data[joint_name] = {}
            x2d = float(line[1])
            y2d = float(line[2])
            x = float(line[3])
            y = float(line[4])
            z = float(line[5])
            self.data[joint_name] = [x,y,z]
        self.frame+=1
        if self.frame == len(self._data_files):
            self.frame=0

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
        int_marker.header.frame_id = "base_link"
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
        int_marker.header.frame_id = "base_link"
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
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = Point(p[0],p[1],3*(p[2]-.25)/4.0)
        int_marker.scale = .45
        int_marker.name = "person_"+name+"_head"
        color = [.7,.7,.7]
        self.makeCirControl(int_marker,color)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
        return int_marker

    def _make_RH(self,p,name):
        # make right hand
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = Point(p[0],p[1],p[2])
        int_marker.scale = 0.15
        int_marker.name = "person_"+name+"_right_hand"
        color = [.5,.5,.5]
        self.makeBoxControl(int_marker,color)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
        return int_marker

    def _make_LH(self,p,name):
        # make right hand
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = Point(p[0],p[1],p[2])
        int_marker.scale = 0.15
        int_marker.name = "person_"+name+"_left_hand"
        color = [.5,.5,.5]
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
    rospy.Timer(rospy.Duration(0.1), P.frameCallback)
    P.server = InteractiveMarkerServer("basic_controls")

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

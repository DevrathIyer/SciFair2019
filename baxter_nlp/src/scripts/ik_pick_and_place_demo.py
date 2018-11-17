#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy
import types
import numpy as np
import rospy
import rospkg
import json
import math
from stanfordcorenlp import StanfordCoreNLP
from fuzzyset import FuzzySet

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class World(object):
    def __init__(self):
	World.objects = ["red block","blue block"]
	World.attributes = {"red block":{"position":Point(x = 0.7, y = 0.15, z = -.129)},"blue block":{"position":Point(x = 0.7 , y = 0.0, z = -.129)}}

world = World()

class nlpBot(object):
    def __init__(self):
        limb = "left"
	self._verbose = False
	self.function_names = ["approach","grip","release","reset"]
	self.functions = {"reset":[self.move_to_start],"approach":[self._approach,self._servo_to_pose],"grip": [self.gripper_open],"release": [self.gripper_close]}
	self._nlp_properties = {"annotators": "tokenize,ssplit,pos,openie", "outputFormat": "json"}
    	self._nlp = StanfordCoreNLP('http://corenlp.run', port=80)
        self._limb_name = "left" # string\	
	self._hover_distance = 0.08
	self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self,obj = None):
	start_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self,obj = None):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self,obj = None):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, obj):
	overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
	pose = Pose(position = obj["position"],orientation=overhead_orientation)
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self,obj):
        # servo down to release
	overhead_orientation = Quaternion(
                             x=0,
                             y=1,
                             z=0,
                             w=0)
	pose = Pose(position = obj["position"],orientation=overhead_orientation)
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)
   
    def execute(self,command):
	ReturnedJson = json.loads(self._nlp.annotate(command,properties=self._nlp_properties))['sentences'][0]
	dependencies = []
        for dependency in ReturnedJson['enhancedPlusPlusDependencies']:
		dependencies.append((dependency['governorGloss'],dependency['dependentGloss'],dependency['dep']))
	verb_tuple = [x for x in dependencies if x[2] == u'dobj'][0]
	functions = FuzzySet(self.function_names)
	rospy.logerr(verb_tuple[0])
	query = functions.get(str(verb_tuple[0]))
	obj = verb_tuple[1]
        obj_adj = str(" ".join([x[1] for x in dependencies if x[0] == obj and not x[2] == u'det']) + " " + obj)
        objects = FuzzySet(world.objects)
        object_query = objects.get(obj_adj)
	if query:
	    if query[0] < .5:
	        self._learn(command)
	    else:
		query = query[0]
	        funcs = self.functions.get(query[1])
	        if object_query:
		    for func in funcs:
		    	func(world.attributes.get(object_query[0][1]))
	        else:
		    rospy.logerr("no objects found!")
		    for func in funcs:
		    	func()
    	else:
	    self._learn(verb_tuple[0])
    
    def _learn(self,command):
	functions = FuzzySet(self.function_names)
	rospy.logerr("No command found! Please input commands")
	self.function_names.append(command)
	self.functions[command] = []
	commands = raw_input()
	sentences = json.loads(self._nlp.annotate(commands,properties=self._nlp_properties))['sentences']
	for sentence in sentences:
	    dependencies = []
            for dependency in sentence['enhancedPlusPlusDependencies']:
		dependencies.append((dependency['governorGloss'],dependency['dependentGloss'],dependency['dep']))
	    verb_tuple = [x for x in dependencies if x[2] == u'dobj'][0]
	    query = functions.get(str(verb_tuple[0]))
	    if query:
	        query = query[0]
                func = self.functions.get(query[1])
	        self.functions[command].extend(func)
	rospy.logerr(self.functions)

def main():
    #init node
    rospy.init_node("ik_pick_and_place_demo",log_level=rospy.DEBUG)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters

    pnp = nlpBot()
    pnp.move_to_start()
    rospy.logerr("READY FOR COMMANDS")
    while not rospy.is_shutdown():
    	cmd = raw_input()
	pnp.execute(cmd)
    return 0

if __name__ == '__main__':
    sys.exit(main())

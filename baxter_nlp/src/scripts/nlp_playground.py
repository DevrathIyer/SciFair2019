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
import inspect
import argparse
import struct
import sys
import copy
import types
import numpy as np
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
class World(object):
    def __init__(self):
	World.objects = ["red block","blue block"]
	World.attributes = {"red block":{"position":Point(x = 0, y = 0, z = 0)}}

world = World()

class nlpBot(object):
    def __init__(self):
	self.function_names = ["null","approach","open gripper","close gripper"]
	self.functions = {"null":compile("""start_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}\nself.move_to_start(start_angles)""", 'test', 'exec'),"approach":self._approach,"open gripper": self.gripper_open,"close gripper": self.gripper_close}
	self._nlp_properties = {"annotators": "tokenize,ssplit,pos,openie", "outputFormat": "json"}
    	self._nlp = StanfordCoreNLP('http://corenlp.run', port=80)

    def gripper_open(self,obj):
        print("Gripper close")

    def gripper_close(self,obj):
        print("Gripper open")

    def _approach(self, obj):
	print(obj["position"])
   
    def execute(self,command):
	ReturnedJson = json.loads(self._nlp.annotate(command,properties=self._nlp_properties))['sentences'][0]
	dependencies = []
        for dependency in ReturnedJson['enhancedPlusPlusDependencies']:
		dependencies.append((dependency['governorGloss'],dependency['dependentGloss'],dependency['dep']))
	verb_tuple = [x for x in dependencies if x[2] == u'dobj'][0]
	functions = FuzzySet(self.function_names)
	query = functions.get(str(verb_tuple[0]))
	if query:
	    query = query[0]
	    print(query)
	    if query[0] < .5:
	        print("learning")
	        self._learn(command)
	    else:
	        dyn = self.functions.get(query[1])
		obj = verb_tuple[1]
		obj_adj = str(" ".join([x[1] for x in dependencies if x[0] == obj and not x[2] == u'det']) + " " + obj)
		objects = FuzzySet(world.objects)
		object_query = objects.get(obj_adj)
		if object_query:
		    dyn(world.attributes.get(object_query[0][1]))
		else:
		    print("no objects found!")
	else:
	    self._learn(command)
    
    def _learn(self,command):
	fs = FuzzySet(self.function_names)
	fs.add(command)
	print("No command found! Please input commands")
	commands = raw_input()
	commandArray = commands.split('.')

def main():
    pnp = nlpBot()
    # An orientation for gripper fingers to be overhead and parallel to the obj
    while True:
    	cmd = raw_input()
	pnp.execute(cmd)
    return 0

if __name__ == '__main__':
    sys.exit(main())

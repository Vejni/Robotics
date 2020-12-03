#!/usr/bin/env python

"""
This script reads the config.yaml and sets the parameters in ros then quits
"""

import rospy
import yaml
import os


def read_config():
	try:
		with open("./world/config.yaml") as conf:
			stream = yaml.safe_load(conf)
			for param in stream.keys():
				print(param, stream[param])
				rospy.set_param(param, stream[param])
	except Exception as e:
		print("Error reading config: ", e)


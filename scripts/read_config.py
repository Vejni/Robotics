#!/usr/bin/env python

"""
This script reads the config.yaml and sets the parameters in ros then quits
"""

import rospy
import yaml


if __name__ == "__main__":
	with open("../world/config.yaml") as conf:
		try:
			stream = yaml.safe_load(conf)
			for param in stream.keys():
				print(param, stream[param])
				rospy.set_param(param, stream[param])
		except Exception as e:
			print("Error reading config: ", e)


#!/usr/bin/env python

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


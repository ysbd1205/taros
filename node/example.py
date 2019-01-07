#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import rospy
import rospkg
r = rospkg.RosPack()
sys.path.append(r.get_path("ryutaro_matsumoto"))
from scripts.move_vs060 import MoveVS060

class Example:
    def __init__(self):
        self.vs060 = MoveVS060()

    def main(self):
        position, orientation = self.vs060.get_current_pose()
        print(position, orientation)
        position[0] -= 0.01
        position[2] += 0.01
        self.vs060.move_to_pose(position, orientation)
        print(position, orientation)

if __name__ == '__main__':
    rospy.init_node("example", anonymous=True)

    e = Example()
    e.main()

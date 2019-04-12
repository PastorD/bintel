#!/usr/bin/env python
import rospy
from scipy.spatial.transform import Rotation as R


r = R.from_euler('zyx',[[10,10,10]],degrees=True)

print(r.as_quat())
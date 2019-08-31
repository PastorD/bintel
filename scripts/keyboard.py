#!/usr/bin/env python

import rospy
import curses
from std_msgs.msg import String
forward = 0;
left = 0;

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('keyboard', String, queue_size=10)

stdscr.refresh()

key = ''
while key != ord('q'):
	key = stdscr.getch()
	stdscr.refresh()
        

        # fill in the conditions to increment/decrement throttle/steer

	if key == curses.KEY_UP:
            forward = 0        
	elif key == curses.KEY_DOWN:
            forward = 0 
	if key == curses.KEY_LEFT:
            left = 0
	elif key == curses.KEY_RIGHT:
            left = 0
        elif key == curses.KEY_DC:
            # this key will center the steer and throttle
            forward = 0
	msg = key
	pub.publish(msg)

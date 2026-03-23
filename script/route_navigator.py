#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class TurtlebotNavigator:
    def __init__(self):
        rospy.init_node('route_navigator')
        self.cur_x, self.cur_y = 0.0, 0.0
        self.pos_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        self.waypoints = [
            {'x': -1.781, 'y': 3.698, 'label': 'L2'},
            {'x': -5.346, 'y': 1.075, 'label': 'L3'},
            {'x': -0.062, 'y': -0.049, 'label': 'L1 (Finish)'}
        ]

    def update_pose(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

    def run(self):
        rospy.sleep(2)
        rospy.loginfo("--- Starting Autonomous Mission ---")
        for pt in self.waypoints:
            rospy.loginfo(f"Moving to target {pt['label']}...")
            goal = PoseStamped()
            goal.header.frame_id, goal.header.stamp = "map", rospy.Time.now()
            goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w = pt['x'], pt['y'], 1.0
            self.goal_pub.publish(goal)
            
            arrived = False
            while not arrived and not rospy.is_shutdown():
                dist = math.sqrt((pt['x'] - self.cur_x)**2 + (pt['y'] - self.cur_y)**2)
                if dist < 0.45:
                    rospy.loginfo(f"Success: Arrived at {pt['label']}")
                    arrived = True
                rospy.sleep(0.5)
            rospy.sleep(2.0)
        rospy.loginfo("--- Mission Complete ---")

    if __name__ == '__main__':
        try:
            nav = TurtlebotNavigator()
            nav.run()
        except rospy.ROSInterruptException: 
            pass
#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest

def main():
    rospy.init_node('delete_waypoints_manual')
    rospy.wait_for_service('/gazebo/delete_model')
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    for i in range(501):  # 0 through 500 inclusive
        model_name = f'waypoint_marker_{i}'
        try:
            delete(model_name)
            rospy.loginfo(f'Deleted: {model_name}')
        except rospy.ServiceException as e:
            rospy.logwarn(f'Could not delete {model_name}: {e}')

if __name__ == '__main__':
    main()

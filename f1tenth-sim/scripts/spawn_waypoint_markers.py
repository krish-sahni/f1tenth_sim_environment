#!/usr/bin/env python3

import rospy
import csv
import os
import threading
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

spawned_ids = []

def delete_models_async():
    rospy.loginfo("Deleting markers...")
    try:
        rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
        delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for model_id in spawned_ids:
            try:
                delete_srv(model_id)
                rospy.loginfo(f"Deleted: {model_id}")
            except Exception as e:
                rospy.logwarn(f"Failed to delete {model_id}: {e}")
    except Exception as e:
        rospy.logwarn(f"Delete service unavailable: {e}")

def shutdown_handler():
    # Spawn a new thread to delete markers quickly
    threading.Thread(target=delete_models_async).start()

def main():
    rospy.init_node('spawn_waypoints_gazebo')
    rospy.on_shutdown(shutdown_handler)

    model_path = rospy.get_param("model_path")
    waypoint_csv = rospy.get_param("waypoint_csv")

    with open(os.path.join(model_path, "model.sdf"), "r") as f:
        model_xml = f.read()

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    with open(waypoint_csv) as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i < 150:
                continue
            x, y = map(float, row)
            pose = Pose(position=Point(x=x, y=y, z=0.05), orientation=Quaternion(0, 0, 0, 1))
            model_name = f"waypoint_marker_{i}"
            try:
                spawn(model_name, model_xml, "", pose, "world")
                spawned_ids.append(model_name)
                rospy.loginfo(f"Spawned: {model_name}")
            except Exception as e:
                rospy.logerr(f"Could not spawn {model_name}: {e}")

    # Keep the node alive
    rospy.spin()


if __name__ == '__main__':
    main()

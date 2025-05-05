#!/usr/bin/env python3

import rospy
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
    rospy.loginfo("Shutting down and deleting model(s)...")
    thread = threading.Thread(target=delete_models_async)
    thread.start()
    thread.join()
    rospy.loginfo("All models deleted.")


def main():
    rospy.init_node('spawn_single_model')
    rospy.on_shutdown(shutdown_handler)

    # Get model path from parameter
    script_dir = os.path.dirname(os.path.realpath(__file__))
    model_path = os.path.join(script_dir, "..", "models", "waypoint_marker")

    # Load model
    with open(os.path.join(model_path, "model.sdf"), "r") as f:
        model_xml = f.read()

    # Coordinates to spawn the model at
    x = rospy.get_param("~x", 1.8101)  # Default to 1.0 if not set
    y = rospy.get_param("~y", 1.6861)
    z = rospy.get_param("~z", 0.0)

    pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(0, 0, 0, 1))

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    model_name = "single_waypoint_marker"

    try:
        spawn(model_name, model_xml, "", pose, "world")
        spawned_ids.append(model_name)
        rospy.loginfo(f"Spawned: {model_name}")
    except Exception as e:
        rospy.logerr(f"Could not spawn {model_name}: {e}")

    rospy.spin()

if __name__ == '__main__':
    main()

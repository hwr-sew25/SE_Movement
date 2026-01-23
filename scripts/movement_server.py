#!/usr/bin/env python3
import rospy
import actionlib
import tf.transformations as tft

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from actionlib_msgs.msg import GoalStatus

from movement_api.srv import GoToPose, GoToPoseResponse


class MovementServer:
    def __init__(self):
        rospy.loginfo("MovementServer startet...")

        # Action-Client für move_base
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Warte auf move_base-Action-Server...")
        self.client.wait_for_server()
        rospy.loginfo("move_base-Server verfügbar.")

        # Service registrieren
        self.service = rospy.Service(
            "/movement/go_to_pose",
            GoToPose,
            self.handle_request
        )
        rospy.loginfo("Service /movement/go_to_pose bereit.")

    def handle_request(self, req):
        x = req.x
        y = req.y
        yaw_deg = req.yaw_deg
        frame = req.frame_id if req.frame_id else "map"

        rospy.loginfo(
            "Goal erhalten: frame_id=%s, x=%.3f, y=%.3f, yaw=%.1f°",
            frame, x, y, yaw_deg
        )

        # Grad -> Radiant -> Quaternion
        yaw_rad = yaw_deg * 3.1415926535 / 180.0
        q = tft.quaternion_from_euler(0.0, 0.0, yaw_rad)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = Quaternion(*q)

        # Goal an move_base schicken
        self.client.send_goal(goal)
        rospy.loginfo("Goal an move_base gesendet, warte auf Ergebnis...")
        self.client.wait_for_result()
        state = self.client.get_state()

        if state == GoalStatus.SUCCEEDED:
            msg = "Ziel erreicht."
            success = True
        else:
            msg = "Ziel NICHT erreicht. State={}".format(state)
            success = False

        rospy.loginfo(msg)
        return GoToPoseResponse(
            success=success,
            state=state,
            message=msg
        )


def main():
    rospy.init_node("movement_server")
    MovementServer()
    rospy.spin()


if __name__ == "__main__":
    main()

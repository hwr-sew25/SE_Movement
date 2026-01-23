#!/usr/bin/env python3
import rospy
import actionlib
import threading
import time

from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

from movement_api.msg import TargetPose, NavStatus


# =========================
# HARD CODED START POSITION
# =========================
START_X = 21.4
START_Y = -26.9
START_YAW = 0.0  # radians


class MovementNode:
    def __init__(self):
        rospy.init_node("movement_node")

        # Topics (can be remapped later via launch)
        self.target_topic = rospy.get_param("~target_topic", "/navbot/target_pose")          # movement_api/TargetPose
        self.status_topic = rospy.get_param("~status_topic", "/navbot/nav_status")          # movement_api/NavStatus
        self.speech_done_topic = rospy.get_param("~speech_done_topic", "/navbot/speech_done")  # std_msgs/Bool
        self.move_base_name = rospy.get_param("~move_base_action", "move_base")

        # Behavior
        # - max_speech_wait_sec: how long we wait for speech_done at target (default 60s)
        # - arrival_delay_sec: extra delay AFTER speech_done (default 10s)
        self.max_speech_wait_sec = float(rospy.get_param("~max_speech_wait_sec", 60.0))
        self.arrival_delay_sec = float(rospy.get_param("~arrival_delay_sec", 10.0))

        # Optional safety: hard timeout per navigation leg
        self.nav_timeout_sec = float(rospy.get_param("~nav_timeout_sec", 180.0))

        # State
        self._speech_done_event = threading.Event()
        self._state_lock = threading.Lock()
        self._busy = False
        self._active_target_id = ""

        # Pub/Sub
        self.status_pub = rospy.Publisher(self.status_topic, NavStatus, queue_size=10)
        rospy.Subscriber(self.speech_done_topic, Bool, self._on_speech_done, queue_size=5)
        rospy.Subscriber(self.target_topic, TargetPose, self._on_target, queue_size=5)

        # move_base client
        self.client = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        rospy.loginfo(f"[Movement] Waiting for action server '{self.move_base_name}'...")
        self.client.wait_for_server()
        rospy.loginfo("[Movement] move_base connected.")

        rospy.loginfo(f"[Movement] Subscribed target: {self.target_topic} (movement_api/TargetPose)")
        rospy.loginfo(f"[Movement] Publishing status: {self.status_topic} (movement_api/NavStatus)")
        rospy.loginfo(f"[Movement] Waiting speech done: {self.speech_done_topic} (std_msgs/Bool)")

        self._publish_status(NavStatus.READY, "", "boot")

    # ----------------
    # Status publishing
    # ----------------
    def _publish_status(self, state, target_id="", detail=""):
        msg = NavStatus()
        msg.header.stamp = rospy.Time.now()
        msg.state = state
        msg.target_id = target_id
        msg.detail = detail
        self.status_pub.publish(msg)
        rospy.loginfo(f"[Movement][STATUS] state={state} target='{target_id}' detail='{detail}'")

    # -------------
    # Sub callbacks
    # -------------
    def _on_speech_done(self, msg: Bool):
        if msg.data:
            self._speech_done_event.set()

    def _on_target(self, msg: TargetPose):
        # Requirement: no abort / ignore new targets while driving
        with self._state_lock:
            if self._busy:
                # Ignore new targets while busy
                self._publish_status(NavStatus.FAILED, msg.target_id, "ignored_busy")
                return
            self._busy = True

        # Run in a thread so callback stays responsive
        t = threading.Thread(
            target=self._execute_cycle,
            args=(msg.target_id, msg.x, msg.y, msg.yaw),
            daemon=True,
        )
        t.start()

    # -----------------
    # move_base helpers
    # -----------------
    def _goal_from_pose(self, x, y, yaw):
        q = quaternion_from_euler(0.0, 0.0, yaw)
        quat = Quaternion(*q)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation = quat
        return goal

    def _send_goal_and_wait(self, x, y, yaw):
        goal = self._goal_from_pose(x, y, yaw)
        self.client.send_goal(goal)

        start = time.time()
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            # Hard timeout
            if (time.time() - start) > self.nav_timeout_sec:
                self.client.cancel_all_goals()
                return "TIMEOUT"

            state = self.client.get_state()
            # actionlib: 3=SUCCEEDED, 4=ABORTED, 5=REJECTED, 2=PREEMPTED, 8=RECALLED, 9=LOST
            if state == 3:
                return "SUCCEEDED"
            if state in (4, 5, 8, 9):
                return f"STATE_{state}"
            if state == 2:
                return "PREEMPTED"

            rate.sleep()

        return "ROS_SHUTDOWN"

    # --------------------
    # Full behavior cycle
    # --------------------
    def _execute_cycle(self, target_id, x, y, yaw):
        self._active_target_id = target_id or ""

        try:
            # Go to target
            self._publish_status(NavStatus.MOVING_TO_TARGET, self._active_target_id, "going")
            res = self._send_goal_and_wait(x, y, yaw)
            if res != "SUCCEEDED":
                self._publish_status(NavStatus.FAILED, self._active_target_id, f"target_failed:{res}")
                return

            # Arrived
            self._publish_status(NavStatus.ARRIVED, self._active_target_id, "arrived")

            # Wait for SpeechOut done (up to max_speech_wait_sec)
            self._publish_status(NavStatus.WAITING_FOR_SPEECH, self._active_target_id, "waiting_speech")
            self._speech_done_event.clear()

            got_speech = self._speech_done_event.wait(timeout=self.max_speech_wait_sec)

            if got_speech:
                # Speech finished -> wait extra 10 seconds before returning
                rospy.loginfo("[Movement] Speech done received -> waiting arrival_delay_sec before returning.")
                rospy.sleep(self.arrival_delay_sec)
                self._publish_status(NavStatus.RETURNING_TO_START, self._active_target_id, "returning_after_speech")
            else:
                # Speech timeout -> return immediately after the 60s wait (no extra 10s)
                rospy.logwarn("[Movement] Speech timeout -> returning immediately.")
                self._publish_status(NavStatus.FAILED, self._active_target_id, "speech_timeout_returning_now")
                self._publish_status(NavStatus.RETURNING_TO_START, self._active_target_id, "returning_after_speech_timeout")

            # Return to start
            res_back = self._send_goal_and_wait(START_X, START_Y, START_YAW)
            if res_back != "SUCCEEDED":
                self._publish_status(NavStatus.FAILED, self._active_target_id, f"return_failed:{res_back}")
                return

            self._publish_status(NavStatus.AT_START, self._active_target_id, "at_start")
            self._publish_status(NavStatus.READY, "", "ready")

        finally:
            with self._state_lock:
                self._busy = False
            self._active_target_id = ""

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    MovementNode().spin()

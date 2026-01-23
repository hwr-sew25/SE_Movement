#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def wait_for_sim_time(timeout_sec: float) -> bool:
    """
    Wait until /clock is running (rospy.Time.now() > 0) when use_sim_time is enabled.
    Uses wall-clock timeout to avoid hanging forever.
    """
    wall_start = rospy.get_time()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() > 0.0:
            return True
        if (rospy.get_time() - wall_start) > timeout_sec:
            return False
        rate.sleep()
    return False

def wait_for_subscriber(pub, timeout_sec: float) -> bool:
    """Wait until at least one subscriber is connected to the publisher."""
    wall_start = rospy.get_time()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if pub.get_num_connections() > 0:
            return True
        if (rospy.get_time() - wall_start) > timeout_sec:
            return False
        rate.sleep()
    return False

def main():
    rospy.init_node("publish_initial_pose", anonymous=False)

    # Pose params
    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    yaw_deg = float(rospy.get_param("~yaw_deg", 0.0))
    frame_id = str(rospy.get_param("~frame_id", "map"))

    # Robustness params
    timeout_sec = float(rospy.get_param("~timeout_sec", 30.0))
    repeat = int(rospy.get_param("~repeat", 10))
    rate_hz = float(rospy.get_param("~rate_hz", 2.0))

    # Topic (für Multi-Robot später gut: namespacen/remappen)
    initialpose_topic = str(rospy.get_param("~initialpose_topic", "/initialpose"))

    pub = rospy.Publisher(initialpose_topic, PoseWithCovarianceStamped, queue_size=1)

    rospy.loginfo(f"[publish_initial_pose] Publishing to: {initialpose_topic}")
    rospy.loginfo("[publish_initial_pose] Waiting for simulation time (/clock) ...")
    if not wait_for_sim_time(timeout_sec):
        rospy.logwarn("[publish_initial_pose] Timeout waiting for /clock. Publishing anyway (AMCL may ignore early messages).")

    rospy.loginfo("[publish_initial_pose] Waiting for AMCL subscriber ...")
    if not wait_for_subscriber(pub, timeout_sec):
        rospy.logwarn("[publish_initial_pose] Timeout waiting for subscriber. Publishing anyway.")
    else:
        rospy.loginfo("[publish_initial_pose] Subscriber detected.")

    yaw = math.radians(yaw_deg)
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    # RViz-typische Kovarianz (grob)
    cov = [0.0] * 36
    cov[0] = 0.25     # x variance
    cov[7] = 0.25     # y variance
    cov[35] = 0.0685  # yaw variance
    msg.pose.covariance = cov

    rate = rospy.Rate(rate_hz)
    for i in range(repeat):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo("[publish_initial_pose] %d/%d: x=%.3f y=%.3f yaw_deg=%.2f frame=%s",
                      i + 1, repeat, x, y, yaw_deg, frame_id)
        rate.sleep()

    rospy.loginfo("[publish_initial_pose] Done.")
    rospy.signal_shutdown("done")

if __name__ == "__main__":
    main()

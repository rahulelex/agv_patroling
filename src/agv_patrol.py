#! /usr/bin/env python
# Required Header files
import rospy, sys, tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped

# Brings in the SimpleActionClient
import actionlib

#
# Patroling Class
#
class Patrol():
    def __init__(self, robot_ns, map_name):
        self.robot_ns = robot_ns
        if not self.robot_ns.endswith("/"):
            self.robot_ns += "/"
        self.map_name = map_name
        self.global_frame = "/" + self.map_name + "/map"
        self.base_frame = self.robot_ns + "base_link"
        self.transformer = tf.TransformListener()
        self.goal_patrol = None
        self.goal_receive = rospy.Subscriber("/patrol", PoseStamped, self.agvPatrolHandler)
        self.pose_array_publisher = rospy.Publisher(self.robot_ns+'initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.total_round = 0
        rospy.loginfo('*****Patroling Server Started*****')

    # Initiate robot patroling
    def startPatroling(self):
        while not rospy.is_shutdown():
            if self.goal_patrol is not None:
                rospy.logdebug('***Patroling Started***')
                self.agvPatrol(self.goal_patrol)

    #
    # Return the robot's current location as an associative array with x, y, and a keys
    #
    def getLocation(self, global_frame=None, base_frame=None):
        if global_frame == None:
            global_frame = self.global_frame
        if base_frame == None:
            base_frame = self.base_frame
        cond = 0;
        while cond == 0:
            try:
                (trans,rot) = self.transformer.lookupTransform(global_frame, base_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0

        euler = tf.transformations.euler_from_quaternion(rot)
        return (trans[0], trans[1], euler[2])

    #
    # Create second goal pose
    #
    def createGoalPose2(self, goal_pose1):
        goal_pose2 = PoseStamped()
        goal_pose2.header.seq = goal_pose1.header.seq + 1
        goal_pose2.header.frame_id = self.global_frame[1:]
        goal_pose2.header.stamp = rospy.Time.now()
        x,y,a = self.getLocation()
        goal_pose2.pose.position.x = x
        goal_pose2.pose.position.y = y
        q_array = tf.transformations.quaternion_from_euler(0, 0, a, axes='sxyz')
        goal_pose2.pose.orientation = Quaternion(*q_array)
        return goal_pose2

    #
    # Publish robot's pose array
    #
    def poseArray(self):
        rospy.loginfo('Publishing pose array')

        current = self.getLocation()
        pose_array = PoseWithCovarianceStamped()
        pose_array.header.frame_id = self.global_frame[1:]
        pose_array.header.stamp = rospy.Time.now()
        pose_array.pose.pose.position.x = current[0]
        pose_array.pose.pose.position.y = current[1]
        q_array = tf.transformations.quaternion_from_euler(0, 0, current[2], axes='sxyz')
        pose_array.pose.pose.orientation = Quaternion(*q_array)
        pose_array.pose.covariance= [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

        rospy.logdebug('Pose array : %s' %(pose_array))
        self.pose_array_publisher.publish(pose_array)

    #
    # Navigate robot to goal
    #
    def navigateAgv(self, pose):
        max_wait = 120
        # Creates the SimpleActionClient, passing the type of the action
        # (MoveBaseAction) to the constructor.
        client = actionlib.SimpleActionClient(self.robot_ns+'move_base', MoveBaseAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = MoveBaseGoal()
        goal.target_pose = pose

        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.loginfo('Goal sent')
        rospy.logdebug('Goal sent is: %s' %(goal))

        # Waits for the server to finish performing the action.
        client.wait_for_result(rospy.Duration(10.0))

        while max_wait > 0:
            result = client.get_state()
            if result >= 2 and result <= 5:
                return result == 3
            if max_wait % 10 ==0:
                rospy.logdebug(result)
            rospy.sleep(1)
            max_wait -=1
        # Prints out the result of executing the action
        return client.get_result()  # NavigationResult

    #
    # Update patroling goal
    #
    def agvPatrolHandler(self, goal_pose):
        rospy.loginfo('Received New Goal')
        rospy.logdebug('New goal is: %s' %goal_pose)
        self.goal_patrol = goal_pose

    #
    # Start patroling from initial point to patroling point
    #
    def agvPatrol(self, goal_pose1):
        goal_pose2 = self.createGoalPose2(goal_pose1)
        rospy.logdebug('Patroling Started From: %s' %goal_pose1)
        rospy.logdebug('Patroling Started To: %s' %goal_pose2)
        self.poseArray()
        self.navigateAgv(goal_pose1)
        self.poseArray()
        self.navigateAgv(goal_pose2)
        self.total_round += 1
        rospy.loginfo('Total complete round completed = %s' %(self.total_round))


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('invalid argument')
        sys.exit()
    try:
        # Initializes a Patroling Node
        rospy.init_node('agv_patrol',anonymous=False, log_level=rospy.DEBUG)
        p = Patrol(sys.argv[1], sys.argv[2])
        p.startPatroling()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
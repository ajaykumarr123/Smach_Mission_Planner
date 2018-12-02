import rospy
#from __future__ import print_function
import smach
import actionlib
import kraken_msgs.msg
import tf
# define state Foo
class Depth(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['surge','stop1'])
     

    def execute(self, userdata):
        rospy.loginfo('Executing state Depth')
        _actionClient = actionlib.SimpleActionClient('CONTROL_ADVANCEDCONTROLLER_ACTION', kraken_msgs.msg.advancedControllerAction)
        rospy.loginfo("WAITING FOR CONTROL_SERVER TO START.")
        _actionClient.wait_for_server()
        goal = kraken_msgs.msg.advancedControllerGoal()

        listener=tf.TransformListener()
        transform=tf.StampedTransform()
        # convert into python
        """try{
            listener.lookupTransform("base_link", "odom",ros::Time(0), transform);
        }
        catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }"""
        _goal.GoalType = 0
        temp=geometry_msgs.msg.PoseStamped()
        pose=geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "/base_link"
        pose.header.stamp = rospy.Time()
        pose.pose.position.x = 2
        pose.pose.position.y = 0
        pose.pose.position.z = 0# depth???
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        #_state.listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(30), ros::Duration(0.01), "FAILED");
        listener.transformPose("/odom", pose, temp)

        _goal.pose.position.x = temp.pose.position.x 
        _goal.pose.position.y = temp.pose.position.y 
        _goal.pose.position.z = temp.pose.position.z 
        _goal.pose.orientation.x = temp.pose.orientation.x
        _goal.pose.orientation.y = temp.pose.orientation.y
        _goal.pose.orientation.z = temp.pose.orientation.z
        _goal.pose.orientation.w = temp.pose.orientation.w
        _actionClient.send_goal(_goal)
        _actionClient.wait_for_result()
        result = _actionClient.get_result()
        if result ==1:
            return surge
        else:
            return stop

# define state Bar
class Surge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Surge')
        actionClient = actionlib.SimpleActionClient('CONTROL_ADVANCEDCONTROLLER_ACTION', kraken_msgs.msg.advancedControllerAction)
        rospy.loginfo("WAITING FOR CONTROL_SERVER TO START.")
        _actionClient.wait_for_server()
        goal = kraken_msgs.msg.advancedControllerGoal()

        listener=tf.TransformListener()
        transform=tf.StampedTransform()
        # convert into python
        """try{
            listener.lookupTransform("base_link", "odom",ros::Time(0), transform);
        }
        catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }"""
        _goal.GoalType = 0
        temp=geometry_msgs.msg.PoseStamped()
        pose=geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "/base_link"
        pose.header.stamp = rospy.Time()
        pose.pose.position.x = 2#surge??
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        #_state.listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(30), ros::Duration(0.01), "FAILED");
        listener.transformPose("/odom", pose, temp)

        _goal.pose.position.x = temp.pose.position.x 
        _goal.pose.position.y = temp.pose.position.y 
        _goal.pose.position.z = temp.pose.position.z 
        _goal.pose.orientation.x = temp.pose.orientation.x
        _goal.pose.orientation.y = temp.pose.orientation.y
        _goal.pose.orientation.z = temp.pose.orientation.z
        _goal.pose.orientation.w = temp.pose.orientation.w
        _actionClient.send_goal(_goal)
        _actionClient.wait_for_result()
        result = _actionClient.get_result()
        if result ==1:
            return stop
        else:
            return stop
        
# main
def main():
    rospy.init_node('Test_smach')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Depth', Depth(), 
                               transitions={'surge':'Surge', 
                                            'stop1':'outcome4'})
        smach.StateMachine.add('Surge', Surge(), 
                               transitions={'stop2':'outcome4'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
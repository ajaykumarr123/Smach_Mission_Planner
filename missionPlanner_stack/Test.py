import rospy
#from __future__ import print_function
import smach
import actionlib
import kraken_msgs.msg
import tf
import tf2_ros
import smach_ros
# define state Depth


import geometry_msgs.msg
rospy.init_node('Test_smach')
listener = tf.TransformListener()   
try:
	(trans) = listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	rospy.loginfo("error")
class Depth(smach.State):
	def __init__(self):
        	smach.State.__init__(self, outcomes=['surge','stop1'])
     

   	def execute(self, userdata):
		rospy.loginfo('Executing state Depth')
		_actionClient = actionlib.SimpleActionClient("/kraken/control/advancedcontrol_action", kraken_msgs.msg.advancedControllerAction)
		rospy.loginfo("WAITING FOR CONTROL_SERVER TO START.")
		_actionClient.wait_for_server()
		rospy.loginfo("CONTROL_SERVER TO STARTed.")
		_goal = kraken_msgs.msg.advancedControllerGoal()   
		_goal.GoalType = 0
		temp=geometry_msgs.msg.PoseStamped()
		pose=geometry_msgs.msg.PoseStamped()
		pose.header.frame_id = "/base_link"
		pose.header.stamp = rospy.Time()
		pose.pose.position.x = 0
		pose.pose.position.y = 0
		pose.pose.position.z = 2
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = 1
		temp=listener.transformPose("odom",pose)
		_goal.pose.position.x = temp.pose.position.x 
		_goal.pose.position.y = temp.pose.position.y 
		_goal.pose.position.z = temp.pose.position.z
		_goal.pose.orientation.x = temp.pose.orientation.x
		_goal.pose.orientation.y = temp.pose.orientation.y
		_goal.pose.orientation.z = temp.pose.orientation.z
		_goal.pose.orientation.w = temp.pose.orientation.w
		_actionClient.send_goal(_goal)
		_actionClient.wait_for_result(rospy.Duration.from_sec(20))
		result = _actionClient.get_result()
		if result !=0:
		    rospy.loginfo(result)
		    return 'surge'
		else:
		    return 'stop1'

# define state Surge
class Line_Follow(smach.State):
	def __init__(self):
            smach.State.__init__(self, outcomes=['line_succeed','stop2'])
	#def execute(self):

        #sent goal to line follower 
        # subcribe the to topic published by line_following
        #sent goal to controller to achive it 
        #goal succeed by line follower achived by IP server 
        #stop controller 
class Buoy_detection(smach.State):
    def __init__(self):
    	smach.State.__init__(self, outcomes=["buoy_succeed","stop3"])
	#def execute(self):
       #sent goal to buoy detector
        # subcribe the to topic published by buoy_detector
        #sent goal to controller to achive it 
        #goal succeed by buoy detctor  achived by IP server 
        #stop controller 

class Gate_task(smach.State):
	def __init__(self):
            smach.State.__init__(self, outcomes=["gate_succeed","stop4"])
	#def execute(self):
       #sent goal to gate detector
        # subcribe the to topic published by gate_detector
        #sent goal to controller to achive it 
        #goal succeed by gate detetor  achived by IP server 
        #stop controller 

class Surge(smach.State):
	def __init__(self):
            smach.State.__init__(self, outcomes=['line_follow','stop5'])

	def execute(self):
		rospy.loginfo('Executing state Surge')
		_actionClient = actionlib.SimpleActionClient("/kraken/control/advancedcontrol_action", kraken_msgs.msg.advancedControllerAction)
		rospy.loginfo("WAITING FOR CONTROL_SERVER TO START.")
		_actionClient.wait_for_server()
		_goal = kraken_msgs.msg.advancedControllerGoal()
		_goal.GoalType = 0
		temp=geometry_msgs.msg.PoseStamped()
		pose=geometry_msgs.msg.PoseStamped()
		pose.header.frame_id = "/base_link"
		pose.header.stamp = rospy.Time()
		pose.header.stamp = rospy.Time()
		pose.pose.position.x = 2
		pose.pose.position.y = 0
		pose.pose.position.z = 0# depth???
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = 1
		temp=listener.transformPose("odom",pose)
		_goal.pose.position.x = temp.pose.position.x 
		_goal.pose.position.y = temp.pose.position.y 
		_goal.pose.position.z = temp.pose.position.z
		_goal.pose.orientation.x = temp.pose.orientation.x
		_goal.pose.orientation.y = temp.pose.orientation.y
		_goal.pose.orientation.z = temp.pose.orientation.z
		_goal.pose.orientation.w = temp.pose.orientation.w
		_actionClient.send_goal(_goal)
		_actionClient.wait_for_result(rospy.Duration.from_sec(25))
		result = _actionClient.get_result()
		if result ==1:
		    return 'stop2'
		else:
		    return 'stop2'
        
# main
def main():
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Depth', Depth(), 
                               transitions={'surge':'Surge', 
                                            'stop1':'outcome4'})
        smach.StateMachine.add('Surge', Surge(), 
                               transitions={'line_follow':'Line_Follow',
                               'stop5':'outcome4'})
        smach.StateMachine.add('Line_Follow', Line_Follow(), 
                               transitions={'line_succeed':'Buoy_detection','stop2':'outcome4'})
        smach.StateMachine.add('Buoy_detection', Buoy_detection(), 
                               transitions={'buoy_succeed':'Line_Follow1','stop3':'outcome4'})
        smach.StateMachine.add('Line_Follow1', Line_Follow(), 
                               transitions={'line_succeed':'Gate_task','stop2':'outcome4'})
        smach.StateMachine.add('Gate_task',Gate_task(),
            transitions={'gate_succeed':'outcome4','stop4':'outcome4'})
    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
    sis.start()
    # Execute the state machine
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
   


if __name__ == '__main__':
    main()

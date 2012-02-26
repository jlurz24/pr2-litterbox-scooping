import roslib; roslib.load_manifest('litterbox')
import rospy
import smach
import smach_ros
from litterbox.msg import MoveToPositionAction
from smach_ros import SimpleActionState, MonitorState
from litterbox.msg import MoveToPositionGoal
from geometry_msgs.msg import PointStamped

# define state DetectLitterbox
class DetectLitterbox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['litterbox_detected','litterbox_not_detected'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DetectLitterbox')
        # Listen for messages here
        return 'litterbox_detected';

# main
def main():
    rospy.init_node('scoop_litterbox_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    sis = smach_ros.IntrospectionServer('litterbox', sm, '/LITTERBOX_ROOT')
    sis.start()
    
    # Open the container
    with sm:
        
        def move_to_litterbox_cb(userdata, goal):
          goal.position = userdata.litterbox_position
          return goal

        def detect_cb(userdata, message):
          rospy.loginfo('message: ' + message)
          userdata.litterbox_position = message.position
          return false # proceed

        # Add states to the container
        smach.StateMachine.add('DETECT_LITTERBOX', MonitorState('litterbox_location', PointStamped, detect_cb),
                               transitions={'valid':'MOVE_TO_LITTERBOX', 
                                            'preempted': 'DETECT_LITTERBOX',
                                            'invalid':'failure'})
        smach.StateMachine.add('MOVE_TO_LITTERBOX', 
                               SimpleActionState('move_to_position',
                               MoveToPositionAction,
                               goal_cb=move_to_litterbox_cb,
                               input_keys=[]), 
                               transitions={'succeeded':'success',
                                            'preempted':'failure',
                                            'aborted': 'failure'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

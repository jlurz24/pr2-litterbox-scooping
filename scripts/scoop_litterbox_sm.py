import roslib; roslib.load_manifest('litterbox')
import rospy
import smach
import smach_ros
import traceback

from litterbox.msg import MoveToPositionAction
from smach_ros import SimpleActionState, MonitorState
from litterbox.msg import MoveToPositionGoal
from geometry_msgs.msg import PointStamped

# main
def main():
    rospy.init_node('scoop_litterbox_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    sm.userdata.litterbox_position = None

    sis = smach_ros.IntrospectionServer('litterbox', sm, '/LITTERBOX_ROOT')
    sis.start()
    
    # Open the container
    with sm:
        
        def move_to_litterbox_cb(userdata, goal):
          rospy.loginfo("Inside move to lb callback")
          goal = MoveToPositionGoal()
          goal.position.point = userdata.litterbox_position
          return goal

        def detect_cb(userdata, msg):
          rospy.loginfo("Inside detection callback")
          sm.userdata.litterbox_position = msg.point
          return False

        # Add states to the container
        smach.StateMachine.add('DETECT_LITTERBOX', 
                                MonitorState('object_location/Litterbox', PointStamped, detect_cb, 100), 
                                transitions = {'valid':'failure', 
                                               'preempted': 'DETECT_LITTERBOX',
                                               'invalid':'MOVE_TO_LITTERBOX'})

        smach.StateMachine.add('MOVE_TO_LITTERBOX', 
                               SimpleActionState('move_to_position',
                               MoveToPositionAction,
                               goal_cb=move_to_litterbox_cb,
                               input_keys=['litterbox_position']), 
                               transitions={'succeeded':'success',
                                            'preempted':'failure',
                                            'aborted': 'failure'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

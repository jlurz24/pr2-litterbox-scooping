import roslib; roslib.load_manifest('litterbox')
import rospy
import smach
import smach_ros
import traceback

from litterbox.msg import MoveToPositionAction
from smach_ros import SimpleActionState, MonitorState
from litterbox.msg import MoveToPositionGoal
from geometry_msgs.msg import PointStamped
from litterbox.msg import ScoopLitterboxAction
from litterbox.msg import ScoopLitterboxGoal

# main
def main():
    rospy.init_node('scoop_litterbox_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    sm.userdata.litterbox_position = None
    sm.userdata.trash_position = None

    sis = smach_ros.IntrospectionServer('litterbox', sm, '/LITTERBOX_ROOT')
    sis.start()
    
    # Open the container
    with sm:
        
        def move_to_litterbox_cb(userdata, goal):
          rospy.loginfo("Inside move to lb callback")
          goal = MoveToPositionGoal()
          goal.position.point = userdata.litterbox_position
          return goal
 
        def move_to_trash_cb(userdata, goal):
          rospy.loginfo("Inside move to trash callback")
          goal = MoveToPositionGoal()
          goal.position.point = userdata.trash_position
          return goal

        
        def detect_litterbox_cb(userdata, msg):
          rospy.loginfo("Inside litterbox detection callback")
          sm.userdata.litterbox_position = msg.point
          return False
        
        def detect_trash_cb(userdata, msg):
          rospy.loginfo("Inside trash detection callback")
          sm.userdata.trash_position = msg.point
          return False


        def scoop_litterbox_cb(userdata, goal):
          rospy.loginfo("Inside scoop litterbox callback")
          goal = ScoopLitterboxGoal()
          return goal

        # Add states to the container
        smach.StateMachine.add('DETECT_LITTERBOX', 
                                MonitorState('object_location/Litterbox', PointStamped, detect_litterbox_cb, 100), 
                                transitions = {'valid':'failure', 
                                               'preempted': 'DETECT_LITTERBOX',
                                               'invalid':'MOVE_TO_LITTERBOX'})

        smach.StateMachine.add('MOVE_TO_LITTERBOX', 
                               SimpleActionState('move_to_position',
                               MoveToPositionAction,
                               goal_cb=move_to_litterbox_cb,
                               input_keys=['litterbox_position']), 
                               transitions={'succeeded':'SCOOP_LITTERBOX',
                                            'preempted':'failure',
                                            'aborted': 'failure'})
       
        smach.StateMachine.add('SCOOP_LITTERBOX',
                               SimpleActionState('scoop_litterbox',
                               ScoopLitterboxAction,
                               goal_cb=scoop_litterbox_cb,
                               input_keys=[]),
                               transitions={'succeeded':'MOVE_TO_TRASH',
                                            'preempted':'failure',
                                            'aborted': 'failure'})

            # Add states to the container
        smach.StateMachine.add('DETECT_TRASH',
                                MonitorState('object_location/Trash', PointStamped, detect_trash_cb, 100),
                                transitions = {'valid':'failure',
                                               'preempted': 'DETECT_TRASH',
                                               'invalid':'MOVE_TO_TRASH'})

        smach.StateMachine.add('MOVE_TO_TRASH',
                               SimpleActionState('move_to_position',
                               MoveToPositionAction,
                               goal_cb=move_to_trash_cb,
                               input_keys=['trash_position']),
                               transitions={'succeeded':'success',
                                            'preempted':'failure',
                                            'aborted': 'failure'})
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

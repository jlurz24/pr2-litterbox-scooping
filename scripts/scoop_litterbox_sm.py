import roslib
roslib.load_manifest('litterbox')
import rospy, smach, smach_ros, traceback, litterbox.msg

from geometry_msgs.msg import PointStamped
from smach_ros import MonitorState, SimpleActionState
from litterbox.msg import ExploreAction, MoveToPositionAction, ScoopLitterboxAction, DumpPoopAction, InitAction, InsertScooperAction
from litterbox.msg import MoveToPositionGoal, ScoopLitterboxGoal, ExploreGoal, DumpPoopGoal, InitGoal, InsertScooperGoal
from litterbox.msg import ScooperAttached

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
          goal.position.point = sm.userdata.litterbox_position
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

        def detect_scooper_attached_cb(userdata, msg):
            return msg.attached

        def scoop_litterbox_cb(userdata, goal):
          rospy.loginfo("Inside scoop litterbox callback")
          goal = ScoopLitterboxGoal()
          return goal

        def dump_poop_cb(userdata, goal):
          rospy.loginfo("Inside dump poop callback")
          goal = DumpPoopGoal()
          return goal

        def explore_for_litterbox_cb(userdata, goal):
          rospy.loginfo("Inside explore for lb cb")
          goal = ExploreGoal()
          return goal

        def explore_for_trash_cb(userdata, goal):
          rospy.loginfo("Inside explore for trash cb")
          goal = ExploreGoal()
          return goal

        def stop_action_cb(outcomes):
          rospy.loginfo("Inside child termination")
          return True

        def init_action_cb(userdata, goal):
          rospy.loginfo("Inside init action cb")
          goal = InitGoal()
          return goal

        def insert_scooper_cb(userdata, goal):
            rospy.loginfo("Inside insert scooper cb")
            goal = InsertScooperGoal()
            return goal
        

        # Add states to the container
        smach.StateMachine.add('INIT',
                               SimpleActionState('init',
                               InitAction,
                               goal_cb=init_action_cb,
                               input_keys=[]),
                               transitions={'succeeded':'CON_DETECT_LITTERBOX',
                                            'preempted':'INIT',
                                            'aborted': 'failure'})


        # Create the sub SMACH state machine
        sm_con_detect_litterbox = smach.Concurrence(
          outcomes=['detected','notdetected'],
          default_outcome='notdetected',
          outcome_map={
            'detected':
              { 'DETECT_LITTERBOX':'invalid'},
            'notdetected':
              {'EXPLORE_FOR_LB':'succeeded', 'DETECT_LITTERBOX':'valid'}
            },
          child_termination_cb = stop_action_cb
        )

        with sm_con_detect_litterbox:

          smach.Concurrence.add('DETECT_LITTERBOX',
                                MonitorState('object_location/Litterbox', PointStamped, detect_litterbox_cb))

          smach.Concurrence.add('EXPLORE_FOR_LB',
                               SimpleActionState('explore',
                               ExploreAction,
                               goal_cb=explore_for_litterbox_cb,
                               input_keys=[]))


        smach.StateMachine.add('CON_DETECT_LITTERBOX', sm_con_detect_litterbox,
                               transitions={'detected':'MOVE_TO_LITTERBOX',
                                            'notdetected':'failure'})

        sm_con_move_to_litterbox = smach.Concurrence(
          outcomes=['reached','notreached','scooper_dropped'],
          default_outcome='notreached',
          outcome_map={
            'scooper_dropped': {'DETECT_SCOOPER_ATTACHED':'invalid'},
            'reached': { 'MOVE_TO_LITTERBOX_INTERNAL':'succeeded'},
            'notreached': {'MOVE_TO_LITTERBOX_INTERNAL':'aborted'}
            },
          child_termination_cb = stop_action_cb
        )

        with sm_con_move_to_litterbox:
            smach.Concurrence.add('MOVE_TO_LITTERBOX_INTERNAL',
                                   SimpleActionState('move_to_position',
                                   MoveToPositionAction,
                                   goal_cb=move_to_litterbox_cb,
                                   input_keys=[]))
            
            smach.Concurrence.add('DETECT_SCOOPER_ATTACHED',
                                MonitorState('litterbox/scooper_attached', ScooperAttached, detect_scooper_attached_cb))

        smach.StateMachine.add('MOVE_TO_LITTERBOX', sm_con_move_to_litterbox,
                               transitions={'reached':'SCOOP_LITTERBOX',
                                            'notreached':'failure',
                                            'scooper_dropped':'INSERT_SCOOP'})

        smach.StateMachine.add('INSERT_SCOOP',
                               SimpleActionState('insert_scooper',
                                                InsertScooperAction,
                                                goal_cb=insert_scooper_cb,
                                                input_keys=[]),
                                                transitions={'succeeded':'CON_DETECT_LITTERBOX',
                                                             'preempted':'failure',
                                                             'aborted':'failure'})
                               
        smach.StateMachine.add('SCOOP_LITTERBOX',
                               SimpleActionState('scoop_litterbox',
                               ScoopLitterboxAction,
                               goal_cb=scoop_litterbox_cb,
                               input_keys=[]),
                               transitions={'succeeded':'CON_DETECT_TRASH',
                                            'preempted':'failure',
                                            'aborted': 'failure'})

        # Create the sub SMACH state machine
        sm_con_detect_trash = smach.Concurrence(
          outcomes=['detected','notdetected'],
          default_outcome='notdetected',
          outcome_map={
            'detected':
              { 'DETECT_TRASH':'invalid'},
            'notdetected':
              {'EXPLORE_FOR_TRASH':'succeeded', 'DETECT_TRASH':'valid'}
            },
          child_termination_cb = stop_action_cb
        )

        with sm_con_detect_trash:

          smach.Concurrence.add('DETECT_TRASH',
                                MonitorState('object_location/Trash', PointStamped, detect_trash_cb))

          smach.Concurrence.add('EXPLORE_FOR_TRASH',
                               SimpleActionState('explore',
                               ExploreAction,
                               goal_cb=explore_for_trash_cb,
                               input_keys=[]))


        smach.StateMachine.add('CON_DETECT_TRASH', sm_con_detect_trash,
                               transitions={'detected' : 'MOVE_TO_TRASH',
                                            'notdetected' : 'failure'})

        smach.StateMachine.add('MOVE_TO_TRASH',
                               SimpleActionState('move_to_position',
                               MoveToPositionAction,
                               goal_cb=move_to_trash_cb,
                               input_keys=['trash_position']),
                               transitions={'succeeded':'DUMP_POOP',
                                            'preempted':'failure',
                                            'aborted': 'failure'})

        smach.StateMachine.add('DUMP_POOP',
                               SimpleActionState('dump_poop',
                               DumpPoopAction,
                               goal_cb=dump_poop_cb,
                               input_keys=[]),
                               transitions={'succeeded':'CON_DETECT_LITTERBOX',
                                            'preempted':'failure',
                                            'aborted': 'failure'})

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    sis.stop()
    pass


#! /usr/bin/python
import roslib, time
roslib.load_manifest('litterbox')
import rospy, smach, smach_ros, traceback, litterbox.msg

from geometry_msgs.msg import PoseStamped, Quaternion
from smach_ros import MonitorState, SimpleActionState
from litterbox.msg import ExploreAction, MoveToPositionAction, ScoopLitterboxAction, DumpPoopAction, InitAction, InsertScooperAction, FaceTargetAction
from litterbox.msg import MoveToPositionGoal, ScoopLitterboxGoal, ExploreGoal, DumpPoopGoal, InitGoal, InsertScooperGoal, FaceTargetGoal
from litterbox.msg import ScooperAttached
import tf
import math

# Global
transforms = {}

# main
def msg_to_quaternion(msg):
  return [msg.x, msg.y, msg.z, msg.w]

def point_to_vector(point):
  return [point.x, point.y, point.z]

def quaternion_to_msg(q):
  msg = Quaternion()
  msg.x = q[0]
  msg.y = q[1]
  msg.z = q[2]
  msg.w = q[3]
  return msg


def broadcast_timer_callback(event):
  if len(transforms) > 0:
    br = tf.TransformBroadcaster()
    for key, value in transforms.items():
      br.sendTransform(point_to_vector(value.pose.position), msg_to_quaternion(value.pose.orientation), rospy.Time.now(), key, value.header.frame_id)

# TODO: It might be easier just to use setTransform
def broadcast_transform(name, pose):
  transforms[name] = pose

def main():
    rospy.loginfo("Starting the state machine");

    rospy.init_node('scoop_litterbox_state_machine')

    # Start a timer loop for broadcasting transforms
    rospy.Timer(rospy.Duration(0.25), broadcast_timer_callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    sm.userdata.litterbox_pose = None
    sm.userdata.trash_pose = None

    tl = tf.TransformListener()
    sis = smach_ros.IntrospectionServer('litterbox', sm, '/LITTERBOX_ROOT')
    sis.start()
    
    # Open the container
    with sm:

        def move_to_litterbox_cb(userdata, goal):
          rospy.loginfo("Inside move to lb callback")
          goal = MoveToPositionGoal()
          userdata.litterbox_pose.header.stamp = rospy.Time.now()

          # Broadcast the pose as the litterbox_frame
          broadcast_transform('litterbox_frame', userdata.litterbox_pose)
          
          goalInLBFrame = PoseStamped()
          goalInLBFrame.pose.position.y = 0.65
          goalInLBFrame.pose.position.x = -0.2
          goalInLBFrame.pose.position.z = 0
          goalInLBFrame.pose.orientation.x = 0
          goalInLBFrame.pose.orientation.y = 0
          goalInLBFrame.pose.orientation.z = 0
          goalInLBFrame.pose.orientation.w = 1
          goalInLBFrame.header.frame_id = 'litterbox_frame'
          goalInLBFrame.header.stamp = rospy.Time(0)
         
          # Convert to map frame
          rospy.loginfo("Waiting for litterbox transform")
          tl.waitForTransform("/map", goalInLBFrame.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
          goal.target = tl.transformPose("/map", goalInLBFrame)
          goal.target.header.stamp = rospy.Time.now()
 
          goal.pointHeadAtTarget = True
          return goal
       
        def move_away_cb(userdata, goal):
          rospy.loginfo("Inside move away callback")
          goal = MoveToPositionGoal()
          goalInTF = PoseStamped()
          goalInTF.pose.position.y = 0
          goalInTF.pose.position.x = -1
          goalInTF.pose.orientation.x = 0
          goalInTF.pose.orientation.y = 0
          goalInTF.pose.orientation.z = 0
          goalInTF.pose.orientation.w = 1
          goalInTF.header.frame_id = "/torso_lift_link"
          goalInTF.header.stamp = rospy.Time(0)
          
          # Convert to map frame
          rospy.loginfo("Waiting for litterbox transform")
          tl.waitForTransform("/map", goalInTF.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
          goal.target = tl.transformPose("/map", goalInTF)
          goal.target.header.stamp = rospy.Time.now()
          goal.pointHeadAtTarget = False
          return goal
       
        def move_to_trash_cb(userdata, goal):
          rospy.loginfo("Inside move to trash callback")
          
          goal = MoveToPositionGoal()
          userdata.trash_pose.header.stamp = rospy.Time.now()

          # Broadcast the pose as the trash_frame
          broadcast_transform('trash_frame', userdata.trash_pose)
         
          goalInTrashFrame = PoseStamped()
          goalInTrashFrame.pose.position.y = 0
          goalInTrashFrame.pose.position.x = -0.75
          goalInTrashFrame.pose.position.z = 0
          goalInTrashFrame.pose.orientation.x = 0
          goalInTrashFrame.pose.orientation.y = 0
          goalInTrashFrame.pose.orientation.z = 0
          goalInTrashFrame.pose.orientation.w = 1
          goalInTrashFrame.header.frame_id = 'trash_frame'
          goalInTrashFrame.header.stamp = rospy.Time(0)

          # Convert to map frame
          rospy.loginfo("Waiting for trash transform")
          tl.waitForTransform("/map", goalInTrashFrame.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
          goal.target = tl.transformPose("/map", goalInTrashFrame)
          goal.target.header.stamp = rospy.Time.now()
          goal.pointHeadAtTarget = False
          return goal


        def detect_litterbox_cb(userdata, msg):
          rospy.loginfo("Inside litterbox detection callback")
          sm.userdata.litterbox_pose = msg
          return False

        def detect_trash_cb(userdata, msg):
          rospy.loginfo("Inside trash detection callback x %f y %f in map frame", msg.pose.position.x, msg.pose.position.y)
          sm.userdata.trash_pose = msg
          return False

        def detect_scooper_attached_cb(userdata, msg):
            rospy.sleep(1.0)
            return msg.attached

        def scoop_litterbox_cb(userdata, goal):
          rospy.loginfo("Inside scoop litterbox callback")
          goal = ScoopLitterboxGoal()
          userdata.litterbox_pose.header.stamp = rospy.Time()
          tl.waitForTransform("base_link", userdata.litterbox_pose.header.frame_id, userdata.litterbox_pose.header.stamp, rospy.Duration(10.0))

          goal.target = tl.transformPose("base_link", userdata.litterbox_pose)
          rospy.loginfo("Litterbox is at position %f %f", goal.target.pose.position.x, goal.target.pose.position.y)
          return goal

        def dump_poop_cb(userdata, goal):
          rospy.loginfo("Inside dump poop callback")
          goal = DumpPoopGoal()
          userdata.trash_pose.header.stamp = rospy.Time()
          tl.waitForTransform("base_link", userdata.trash_pose.header.frame_id, userdata.trash_pose.header.stamp, rospy.Duration(10.0))
          goal.target = tl.transformPose("base_link", userdata.trash_pose)
          rospy.loginfo("Trash is at position %f %f in base_link prior to scooping", goal.target.pose.position.x, goal.target.pose.position.y)
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
        
        def face_trash_cb(userdata, goal):
          rospy.loginfo("Inside face trash cb")
          goal = FaceTargetGoal()
          goal.target = userdata.trash_pose
          return goal

        def face_lb_cb(userdata, goal):
          rospy.loginfo("Inside face lb cb")
          goal = FaceTargetGoal()
          goal.target = userdata.litterbox_pose
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
                                MonitorState('object_location/Litterbox', PoseStamped, detect_litterbox_cb))

          smach.Concurrence.add('EXPLORE_FOR_LB',
                               SimpleActionState('explore',
                               ExploreAction,
                               goal_cb=explore_for_litterbox_cb,
                               input_keys=[]))


        smach.StateMachine.add('CON_DETECT_LITTERBOX', sm_con_detect_litterbox,
                               transitions={'detected':'FACE_LITTERBOX',
                                            'notdetected':'CON_DETECT_LITTERBOX'})

        smach.StateMachine.add('FACE_LITTERBOX',
                               SimpleActionState('face_target',
                               FaceTargetAction,
                               goal_cb=face_lb_cb,
                               input_keys=['litterbox_pose']),
                               transitions={'succeeded':'DETECT_LITTERBOX_FINAL',
                                            'preempted':'FACE_LITTERBOX',
                                            'aborted': 'FACE_LITTERBOX'})

        smach.StateMachine.add('DETECT_LITTERBOX_FINAL',
                                MonitorState('object_location/Litterbox',
                                             PoseStamped, detect_litterbox_cb),
                                transitions={'invalid':'MOVE_TO_LITTERBOX',
                                             'valid': 'CON_DETECT_LITTERBOX',
                                             'preempted': 'DETECT_LITTERBOX_FINAL'})

        sm_con_move_to_litterbox = smach.Concurrence(
          input_keys=['litterbox_pose'],
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
                                   input_keys=['litterbox_pose']))
            
            smach.Concurrence.add('DETECT_SCOOPER_ATTACHED',
                                MonitorState('litterbox/scooper_attached', ScooperAttached, detect_scooper_attached_cb))

        smach.StateMachine.add('MOVE_TO_LITTERBOX', sm_con_move_to_litterbox,
                               transitions={'reached':'SCOOP_LITTERBOX',
                                            'notreached':'failure',
                                            'scooper_dropped':'INSERT_SCOOP'})

        smach.StateMachine.add('MOVE_AWAY_FROM_LB', 
                               SimpleActionState('move_to_position',
                               MoveToPositionAction,
                               goal_cb=move_away_cb,
                               input_keys=[]),
                               transitions={'succeeded': 'CON_DETECT_TRASH',
                                            'preempted': 'CON_DETECT_TRASH',
                                            'aborted': 'CON_DETECT_TRASH'})

        smach.StateMachine.add('MOVE_AWAY_FROM_TRASH',
                               SimpleActionState('move_to_position',
                               MoveToPositionAction,
                               goal_cb=move_away_cb,
                               input_keys=[]),
                               transitions={'succeeded': 'CON_DETECT_LITTERBOX',
                                            'preempted': 'CON_DETECT_LITTERBOX',
                                            'aborted': 'CON_DETECT_LITTERBOX'})

        smach.StateMachine.add('INSERT_SCOOP',
                               SimpleActionState('insert_scooper',
                                                InsertScooperAction,
                                                goal_cb=insert_scooper_cb,
                                                input_keys=[]),
                                                transitions={'succeeded':'MOVE_TO_LITTERBOX',
                                                             'preempted':'INSERT_SCOOP',
                                                             'aborted':'INSERT_SCOOP'})
                               
        smach.StateMachine.add('SCOOP_LITTERBOX',
                               SimpleActionState('scoop_litterbox',
                               ScoopLitterboxAction,
                               goal_cb=scoop_litterbox_cb,
                               input_keys=['litterbox_pose']),
                               transitions={'succeeded':'MOVE_AWAY_FROM_LB',
                                            'preempted':'SCOOP_LITTERBOX',
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
                                MonitorState('object_location/Trash', PoseStamped, detect_trash_cb))

          smach.Concurrence.add('EXPLORE_FOR_TRASH',
                               SimpleActionState('explore',
                               ExploreAction,
                               goal_cb=explore_for_trash_cb,
                               input_keys=[]))


        smach.StateMachine.add('CON_DETECT_TRASH', sm_con_detect_trash,
                               transitions={'detected' : 'FACE_TRASH',
                                            'notdetected' : 'CON_DETECT_TRASH'})
        
        smach.StateMachine.add('FACE_TRASH',
                               SimpleActionState('face_target',
                               FaceTargetAction,
                               goal_cb=face_trash_cb,
                               input_keys=['trash_pose']),
                               transitions={'succeeded':'DETECT_TRASH_FINAL',
                                            'preempted':'FACE_TRASH',
                                            'aborted': 'FACE_TRASH'})

        smach.StateMachine.add('DETECT_TRASH_FINAL',
                                MonitorState('object_location/Trash',
                                             PoseStamped, detect_trash_cb),
                                transitions={'invalid':'MOVE_TO_TRASH',
                                             'valid': 'CON_DETECT_TRASH',
                                             'preempted': 'DETECT_TRASH_FINAL'})

        smach.StateMachine.add('MOVE_TO_TRASH',
                               SimpleActionState('move_to_position',
                               MoveToPositionAction,
                               goal_cb=move_to_trash_cb,
                               input_keys=['trash_pose']),
                               transitions={'succeeded':'DUMP_POOP',
                                            'preempted':'MOVE_TO_TRASH',
                                            'aborted': 'MOVE_TO_TRASH'})

        smach.StateMachine.add('DUMP_POOP',
                               SimpleActionState('dump_poop',
                               DumpPoopAction,
                               goal_cb=dump_poop_cb,
                               input_keys=['trash_pose']),
                               transitions={'succeeded':'MOVE_AWAY_FROM_TRASH',
                                            'preempted':'MOVE_AWAY_FROM_TRASH',
                                            'aborted': 'MOVE_AWAY_FROM_TRASH'})

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    rospy.loginfo("Scooping movement interupted")
    sis.stop()


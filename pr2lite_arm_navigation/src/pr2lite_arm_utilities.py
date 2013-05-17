#/usr/bin/python

import numpy as np

import roslib; roslib.load_manifest('pr2lite_arm_navigation')
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg  import PoseStamped, Quaternion, Point
from arm_navigation_msgs.msg import (MoveArmAction, MoveArmGoal, 
            PositionConstraint, OrientationConstraint)
from kinematics_msgs.srv import (GetKinematicSolverInfo, GetPositionFK,
            GetPositionFKRequest, GetPositionIK, GetPositionIKRequest, GetConstraintAwarePositionIK, GetConstraintAwarePositionIKRequest )
from pr2_controllers_msgs.msg import (JointTrajectoryAction,
            JointTrajectoryControllerState, SingleJointPositionAction,
            SingleJointPositionGoal, Pr2GripperCommandAction,
            Pr2GripperCommandGoal, JointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance)

from arm_navigation_msgs.srv import (SetPlanningSceneDiff,
            SetPlanningSceneDiffRequest)
from tf import TransformListener, transformations
from tf import transformations as tft

import math
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class PR2Arm():
  

    wipe_started = False 
    standoff = 0.368 #0.2 + 0.168 (dist from wrist to fingertips)
    torso_min = 0.001 #115 
    torso_max = 0.299 #0.295 
    dist = 0. 

    move_arm_error_dict = {
         -1 : "PLANNING_FAILED: Could not plan a clear path to goal.", 
          0 : "Succeeded [0]", 
          1 : "Succeeded [1]", 
         -2 : "TIMED_OUT",
         -3 : "START_STATE_IN_COLLISION: Try freeing the arms manually.",
         -4 : "START_STATE_VIOLATES_PATH_CONSTRAINTS",
         -5 : "GOAL_IN_COLLISION",
         -6 : "GOAL_VIOLATES_PATH_CONSTRAINTS",
         -7 : "INVALID_ROBOT_STATE",
         -8 : "INCOMPLETE_ROBOT_STATE",
         -9 : "INVALID_PLANNER_ID",
         -10 : "INVALID_NUM_PLANNING_ATTEMPTS",
         -11 : "INVALID_ALLOWED_PLANNING_TIME",
         -12 : "INVALID_GROUP_NAME",
         -13 : "INVALID_GOAL_JOINT_CONSTRAINTS",
         -14 : "INVALID_GOAL_POSITION_CONSTRAINTS",
         -15 : "INVALID_GOAL_ORIENTATION_CONSTRAINTS",
         -16 : "INVALID_PATH_JOINT_CONSTRAINTS",
         -17 : "INVALID_PATH_POSITION_CONSTRAINTS",
         -18 : "INVALID_PATH_ORIENTATION_CONSTRAINTS",
         -19 : "INVALID_TRAJECTORY",
         -20 : "INVALID_INDEX",
         -21 : "JOINT_LIMITS_VIOLATED",
         -22 : "PATH_CONSTRAINTS_VIOLATED",
         -23 : "COLLISION_CONSTRAINTS_VIOLATED",
         -24 : "GOAL_CONSTRAINTS_VIOLATED",
         -25 : "JOINTS_NOT_MOVING",
         -26 : "TRAJECTORY_CONTROLLER_FAILED",
         -27 : "FRAME_TRANSFORM_FAILURE",
         -28 : "COLLISION_CHECKING_UNAVAILABLE",
         -29 : "ROBOT_STATE_STALE",
         -30 : "SENSOR_INFO_STALE",
         -31 : "NO_IK_SOLUTION: Cannot reach goal configuration.",
         -32 : "INVALID_LINK_NAME",
         -33 : "IK_LINK_IN_COLLISION: Cannot reach goal configuration\
                                      without contact.",
         -34 : "NO_FK_SOLUTION",
         -35 : "KINEMATICS_STATE_IN_COLLISION",
         -36 : "INVALID_TIMEOUT"
         }

    def __init__(self, arm, tfListener=None):
        self.arm = arm
        if not (self.arm == "right" or self.arm=="left"):
            rospy.logerr("Failed to initialize PR2Arm: \
                        Must pass 'right' or 'left'")
            return
        if tfListener is None:
            self.tf = TransformListener()
        else:
            self.tf = tfListener
        

        if self.arm == "right":
          # PR2: self.arm[0]+ 'arm_controller/joint_trajectory_action',
          arm_controller = "/arm_controllerR/follow_joint_trajectory"
        else:
          arm_controller = "/arm_controller/follow_joint_trajectory"
        self.arm_traj_client = actionlib.SimpleActionClient(
                                arm_controller, 
                                FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for " + arm_controller)
        if self.arm_traj_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Found " + arm_controller)
        else:
            rospy.logwarn("Cannot find " + arm_controller)

        # same for both pr2 and pr2lite
        self.torso_client = actionlib.SimpleActionClient(
                                'torso_controller/position_joint_action',
                                SingleJointPositionAction)
        rospy.loginfo("Waiting for torso_controller/position_joint_action \
                       server")
        if self.torso_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Found torso_controller/position_joint_action server")
        else:
            rospy.logwarn("Cannot find torso_controller/position_joint_action \
                           server")

        self.gripper_client = actionlib.SimpleActionClient(
                                self.arm[0]+'_gripper_controller/gripper_action',
                                Pr2GripperCommandAction)
        rospy.loginfo("Waiting for "+self.arm[0]+
                                "_gripper_controller/gripper_action server")
        if self.gripper_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Found "+self.arm[0]+
                            "_gripper_controller/gripper_action server")
        else:
            rospy.logwarn("Cannot find "+self.arm[0]+
                            "_gripper_controller/gripper_action server")
        

        # rospy.Subscriber('torso_controller/state', 
        #              JointTrajectoryControllerState, self.update_torso_state)
        
        self.log_out = rospy.Publisher(rospy.get_name()+'/log_out', String)
        
        rospy.loginfo("Waiting for "+self.arm.capitalize()+" FK/IK Solver services")
        try:
            rospy.wait_for_service("/pr2lite_"+self.arm+"_arm_kinematics/get_fk")
            rospy.wait_for_service("/pr2lite_"+self.arm+
                                    "_arm_kinematics/get_fk_solver_info")
            rospy.wait_for_service("/pr2lite_"+self.arm+"_arm_kinematics/get_ik")
            rospy.wait_for_service("/pr2lite_"+self.arm+"_arm_kinematics/get_constraint_aware_ik")
            rospy.wait_for_service("/pr2lite_"+self.arm+
                                    "_arm_kinematics/get_ik_solver_info")
            self.fk_info_proxy = rospy.ServiceProxy(
                                "/pr2lite_"+self.arm+
                                "_arm_kinematics/get_fk_solver_info",
                                GetKinematicSolverInfo) 
            self.fk_info = self.fk_info_proxy()
            self.fk_pose_proxy = rospy.ServiceProxy(
                                 "/pr2lite_"+self.arm+"_arm_kinematics/get_fk",
                                 GetPositionFK, True)    
            self.ik_info_proxy = rospy.ServiceProxy(
                                "/pr2lite_"+self.arm+
                                "_arm_kinematics/get_ik_solver_info",
                                GetKinematicSolverInfo)
            self.ik_info = self.ik_info_proxy()
            self.ik_constraint_aware_pose_proxy = rospy.ServiceProxy(
		"/pr2lite_"+self.arm+"_arm_kinematics/get_constraint_aware_ik",
                                GetConstraintAwarePositionIK, True)    
            self.ik_pose_proxy = rospy.ServiceProxy(
		"/pr2lite_"+self.arm+"_arm_kinematics/get_ik",
                                GetPositionIK, True)    
            rospy.loginfo("Found FK/IK Solver services")
        except:
            rospy.logerr("Could not find FK/IK Solver services")
        
        self.set_planning_scene_diff_name= \
                    '/environment_server/set_planning_scene_diff'
        rospy.wait_for_service('/environment_server/set_planning_scene_diff')
        self.set_planning_scene_diff = rospy.ServiceProxy(
                                '/environment_server/set_planning_scene_diff',
                                SetPlanningSceneDiff)
        self.set_planning_scene_diff(SetPlanningSceneDiffRequest())
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)

        self.test_pose = rospy.Publisher("test_pose", PoseStamped)

    def update_joint_state(self, jtcs):
        self.joint_state_time = jtcs.header.stamp
        # ARD
        if len(self.joint_state_pos) == 0 and len(self.ik_info.kinematic_solver_info.joint_names) > 0:
          for joint_state_name in self.ik_info.kinematic_solver_info.joint_names:
              self.joint_state_pos.append(0)
              self.joint_state_vel.append(0)
        i = 0
        for joint in jtcs.name:
          j = 0
          for joint_state_name in self.ik_info.kinematic_solver_info.joint_names:
            if joint == joint_state_name:
              self.joint_state_pos[j] = jtcs.position[i]
              self.joint_state_vel[j] = jtcs.velocity[i]
            j += 1
          if joint == "torso_lift_joint":
            self.torso_state = jtcs.position[i]
          i += 1
        
    def curr_pose(self):

        # (trans,rot) = self.tf.lookupTransform("/torso_lift_link",
        # (trans,rot) = self.tf.lookupTransform("chess_board_raw",
        #                                     # ARD "/"+
        #                                     self.arm+"_wrist_roll_link",
        #                                     rospy.Time(0))
        # print "chess_board_raw curr_pose "
        # print Point(*trans)
        # ARD
        now = rospy.Time.now()
        self.tf.waitForTransform("base_link", self.arm+"_wrist_roll_link", now, rospy.Duration(1.0))
        # (trans,rot) = self.tf.lookupTransform("/torso_lift_link",
        (trans,rot) = self.tf.lookupTransform("base_link",
                                            # ARD "/"+
                                            self.arm+"_wrist_roll_link",
                                            now)
        cp = PoseStamped()
        cp.header.stamp = rospy.Time.now()
        # cp.header.frame_id = '/torso_lift_link'
        cp.header.frame_id = 'base_link'
        cp.pose.position = Point(*trans)
        cp.pose.orientation = Quaternion(*rot)
        return cp
   
    def wait_for_stop(self, wait_time=1, timeout=60):
        rospy.sleep(wait_time)
        end_time = rospy.Time.now()+rospy.Duration(timeout)
        while not self.is_stopped():
            if rospy.Time.now()<end_time:
                rospy.sleep(wait_time)
            else:
                raise

    def is_stopped(self):
        time = self.joint_state_time
        vels = self.joint_state_vel
        if (np.allclose(vels, np.zeros(7)) and
           (rospy.Time.now()-time)<rospy.Duration(0.1)):
            return True
        else:
            return False
    
    def adjust_elbow(self, f32):
        request = self.form_ik_request(self.curr_pose())
        angs =  list(request.ik_request.ik_seed_state.joint_state.position)
        angs[2] += f32.data
        request.ik_request.ik_seed_state.joint_state.position = angs 
        ik_goal = self.ik_pose_proxy(request)
        if ik_goal.error_code.val == 1:
            self.send_joint_angles(ik_goal.solution.joint_state.position)
        else:
            rospy.loginfo("Cannot adjust elbow position further")
            self.log_out.publish(data="Cannot adjust elbow position further")

    def gripper(self, position, effort=30):
        pos = np.clip(position,0.0, 0.09)
        goal = Pr2GripperCommandGoal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.gripper_client.send_goal(goal)
        finished_within_time = self.gripper_client.wait_for_result(
                                                        rospy.Duration(15))
        if not (finished_within_time):
            self.gripper_client.cancel_goal()
            rospy.loginfo("Timed out moving "+self.arm+" gripper")
            return False
        else:
            state = self.gripper_client.get_state()
            result = self.gripper_client.get_result()
            if (state == 3): #3 == SUCCEEDED
                rospy.loginfo("Gripper Action Succeeded")
                return True
            else:
                rospy.loginfo("Gripper Action Failed")
                rospy.loginfo("Failure Result: %s" %result)
                return False


    def pose_relative_trans(self, pose, x=0., y=0., z=0.):
        """Return a pose translated relative to a given pose."""
        ps = deepcopy(pose)
        M_trans = tft.translation_matrix([x,y,z])
        q_ps = [ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]
        M_rot = tft.quaternion_matrix(q_ps)
        trans = np.dot(M_rot,M_trans)
        ps.pose.position.x += trans[0][-1]
        ps.pose.position.y += trans[1][-1]
        ps.pose.position.z += trans[2][-1]
        #print ps
        return ps
    
    def pose_relative_rot(self, pose, r=0., p=0., y=0., degrees=True):
        """Return a pose rotated relative to a given pose."""
        ps = deepcopy(pose) 
        if degrees:
            r = math.radians(r)
            p = math.radians(p)
            y = math.radians(y)
        des_rot_mat = tft.euler_matrix(r,p,y) 
        q_ps = [ps.pose.orientation.x, 
                ps.pose.orientation.y, 
                ps.pose.orientation.z, 
                ps.pose.orientation.w]
        state_rot_mat = tft.quaternion_matrix(q_ps) 
        final_rot_mat = np.dot(state_rot_mat, des_rot_mat) 
        ps.pose.orientation = Quaternion(
                                *tft.quaternion_from_matrix(final_rot_mat))
        return ps
    
    def find_approach(self, pose, standoff=0., axis='x'):
        """Return a PoseStamped pointed down the z-axis of input pose."""
        ps = deepcopy(pose)
        if axis == 'x':
            ps = self.pose_relative_rot(ps, p=90)
            ps = self.pose_relative_trans(ps, -standoff)
        return ps
        
    def calc_dist(self, ps1, ps2):
        """ Return the cartesian distance between the points of 2 poses."""
        p1 = ps1.pose.position
        p2 = ps2.pose.position
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)

    def hand_frame_move(self, x, y=0, z=0, duration=None):
        cp = self.curr_pose()
        newpose = self.pose_relative_trans(cp,x,y,z)
        self.dist = self.calc_dist(cp, newpose)
        return newpose

    def build_trajectory(self, finish, start=None, ik_space=0.10,
                        duration=None, tot_points=None):
        if start == None: # if given one pose, use current position as start, else, assume (start, finish)
            start = self.curr_pose()
            print "start fro curr_pos"
            print start
       
        #TODO: USE TF TO BASE DISTANCE ON TOOL_FRAME MOVEMENT DISTANCE, NOT WRIST_ROLL_LINK

        # Based upon distance to travel, determine the number of intermediate steps required
        # find linear increments of position.

        print "finish pose"
        print finish
        dist = self.calc_dist(start, finish)     #Total distance to travel
        ik_steps = int(round(dist/ik_space)+1.)   
        rospy.loginfo("IK Steps: %s  Distance: %s " %(ik_steps, dist))
        if tot_points is None:
           tot_points = 1500*dist
        if duration is None:
            duration = dist*120
       
        ik_fracs = np.linspace(0, 1, ik_steps) #A list of fractional positions along course to evaluate ik
        tot_points = ik_steps  # ARD
        ang_fracs = np.linspace(0,1, tot_points)  

        seed = None
        x_gap = finish.pose.position.x - start.pose.position.x
        y_gap = finish.pose.position.y - start.pose.position.y
        z_gap = finish.pose.position.z - start.pose.position.z
        print "Arm gaps: x ", x_gap, " y ", y_gap, " z ", z_gap

        qs = [start.pose.orientation.x, start.pose.orientation.y,
              start.pose.orientation.z, start.pose.orientation.w] 
        qf = [finish.pose.orientation.x, finish.pose.orientation.y,
              finish.pose.orientation.z, finish.pose.orientation.w] 

        #For each step between start and finish, find a complete pose (linear position changes, and quaternion slerp)
        steps = [PoseStamped() for i in range(len(ik_fracs))]
        for i,frac in enumerate(ik_fracs):
            steps[i].header.stamp = rospy.Time.now()
            steps[i].header.frame_id = start.header.frame_id
            steps[i].pose.position.x = start.pose.position.x + x_gap*frac
            steps[i].pose.position.y = start.pose.position.y + y_gap*frac
            # steps[i].pose.position.z = start.pose.position.z + z_gap*frac 
            # ARD: Chess hack for torso height.  Really should fix time on tf?
            steps[i].pose.position.z = start.pose.position.z + z_gap*frac - .3
            # steps[i].pose.position.z = start.pose.position.z + z_gap*frac
            new_q = transformations.quaternion_slerp(qs,qf,frac)
            steps[i].pose.orientation.x = new_q[0]
            steps[i].pose.orientation.y = new_q[1]
            steps[i].pose.orientation.z = new_q[2]
            steps[i].pose.orientation.w = new_q[3]
        rospy.loginfo("Planning straight-line path, please wait")
        self.log_out.publish(data="Planning straight-line path, please wait")
        rospy.loginfo("frameid %s %s" % (start.header.frame_id, finish.header.frame_id))
       
        #Find initial ik for seeding
        req = self.form_constraint_aware_ik_request(steps[0])
        ik_goal = self.ik_constraint_aware_pose_proxy(req)
        print "IK goal"
        print req
        # print req.pose_stamped.pose.position
        # print req.ik_seed_state.name
        # print req.ik_seed_state.position
        for i, name in enumerate(ik_goal.solution.joint_state.name):
           rospy.loginfo("traj joints %s" % name)
           if name == 'left_upper_arm_hinge_joint' or name == 'right_upper_arm_hinge_joint':
             hinge_index = i
           elif name == 'left_shoulder_tilt_joint' or name == 'right_shoulder_tilt_joint':
             shoulder_tilt_index = i

        if len(ik_goal.solution.joint_state.position) == 0:
          print "initial seeding failed"
          seed = None
        #ik_points = list([[0]*7 for i in range(len(ik_fracs))])
        ik_points = list()
        ik_fracs2 = list()
        for i, p in enumerate(steps):
            if (i == 0):
              continue
            #request = self.form_ik_request(p)
            request = self.form_constraint_aware_ik_request(p)
            if seed != None:
              request.ik_request.ik_seed_state.joint_state.position = seed
              request.ik_request.ik_seed_state.joint_state.name = ik_goal.solution.joint_state.name
            ik_goal = self.ik_constraint_aware_pose_proxy(request)
            print "IK goal %d" %(i)
            print request
            # print request.pose_stamped.pose.position
            # print request.ik_seed_state.name
            # print request.ik_seed_state.position
            if len(ik_goal.solution.joint_state.position) != 0:
              seed = list(ik_goal.solution.joint_state.position) # seed the next ik w/previous points results
              seed[hinge_index] = -1*seed[shoulder_tilt_index]
              ik_points.append(ik_goal.solution.joint_state.position)
              ik_fracs2.append(ik_fracs[i])
              print "IK solution for %d" % i
            else:
              print "No solution for %d" % i

        rospy.loginfo("linear interp")
        ik_fracs = ik_fracs2
        if len(ik_points) == 0:
          return None

        ik_points = np.array(ik_points)
        # Linearly interpolate angles 10 times between ik-defined points (non-linear in cartesian space, but this is reduced from dense ik sampling along linear path.  Used to maintain large number of trajectory points without running IK on every one.    
        angle_points = np.zeros((7, tot_points))
        for i in xrange(7):
            angle_points[i,:] = np.interp(ang_fracs, ik_fracs, ik_points[:,i])

        #Build Joint Trajectory from angle positions
        traj = JointTrajectory()
        traj.header.frame_id = steps[0].header.frame_id
        traj.joint_names = self.ik_info.kinematic_solver_info.joint_names
        for name in traj.joint_names:
          rospy.loginfo("traj joints %s" % name)
        #print 'angle_points', len(angle_points[0]), angle_points
        for i in range(len(angle_points[0])):
            traj.points.append(JointTrajectoryPoint())
            traj.points[i].positions = angle_points[:,i]
            traj.points[i].velocities = [0]*7
            traj.points[i].time_from_start = rospy.Duration(
                                                ang_fracs[i]*duration)
        return traj

    def build_follow_trajectory(self, traj):
        # Build 'Follow Joint Trajectory' goal from trajectory (includes tolerances for error)
        traj_goal = FollowJointTrajectoryGoal()
        traj_goal.trajectory = traj
        tolerance = JointTolerance()
        tolerance.position = 0.05
        tolerance.velocity = 0.1
        traj_goal.path_tolerance = [tolerance for i in range(len(traj.points))]
        traj_goal.goal_tolerance = [tolerance for i in range(len(traj.points))]
        traj_goal.goal_time_tolerance = rospy.Duration(3)
        return traj_goal

    def follow_trajectory(self, traj_goal):
        self.arm_traj_client.send_goal(traj_goal)
        self.arm_traj_client.wait_for_result(rospy.Duration(200))

    def move_to_tolerance(self, pose):
        attempt = 0
        prevdist = 0
        while True:
          attempt += 1
          print "Attempt %d" % attempt
          traj = self.build_trajectory(pose, None)
          if traj != None:
            goal = self.build_follow_trajectory(traj)
            self.follow_trajectory(goal)
          curpos = self.curr_pose()
          x_gap = pose.pose.position.x - curpos.pose.position.x
          y_gap = pose.pose.position.y - curpos.pose.position.y
          z_gap = pose.pose.position.z - curpos.pose.position.z
          dist = self.calc_dist(curpos, pose)     #Total distance to travel
          print "Arm dist ", dist, " gaps: x ", x_gap, " y ", y_gap, " z ", z_gap
          if abs(x_gap) <= pose.absolute_position_tolerance.x and abs(y_gap) <= pose.absolute_position_tolerance.y and abs(z_gap) <= pose.absolute_position_tolerance.z:
            return
          if abs(dist - prevdist) < .001: # .04 inches
            return
          prevdist = dist



    def move_torso(self, pos):
        rospy.loginfo("Moving Torso to reach arm goal")
        goal_out = SingleJointPositionGoal()
        goal_out.position = pos
        self.torso_client.send_goal(goal_out)
        return True

    def fast_move(self, ps, time=0.):
        ik_goal = self.ik_pose_proxy(self.form_ik_request(ps))
        if ik_goal.error_code.val == 1:
            point = JointTrajectoryPoint()
            point.positions = ik_goal.solution.joint_state.position
            self.send_traj_point(point)
        else:
            rospy.logerr("FAST MOVE IK FAILED!")

    def blind_move(self, ps, duration = None):
        (reachable, ik_goal, duration) = self.full_ik_check(ps)
        if reachable:
            self.send_joint_angles(ik_goal.solution.joint_state.position,
                                   duration)

    def check_torso(self, request):
        """
        For unreachable goals, check to see if moving the torso can solve
        the problem.  If yes, return True, and torso adjustment needed.
        If no, return False, and best possible Torso adjustment.
        """
        goal_z = request.ik_request.pose_stamped.pose.position.z
        torso_pos = self.torso_state.actual.positions[0]
        spine_range = [self.torso_min - torso_pos, self.torso_max - torso_pos]
       
        if abs(goal_z)<0.005:
            #Already at best position, dont try more (avoid moving to noise)
            rospy.loginfo("Torso Already at best possible position")
            return[False, 0]
        #Find best possible case: shoulder @ goal height, max up, or max down. 
        if goal_z >= spine_range[0] and goal_z <= spine_range[1]: 
            rospy.loginfo("Goal within spine movement range")
            request.ik_request.pose_stamped.pose.position.z = 0;
            opt_tor_move = goal_z
        elif goal_z > spine_range[1]:#Goal is above possible shoulder height
            rospy.loginfo("Goal above spine movement range")
            request.ik_request.pose_stamped.pose.position.z -= spine_range[1]
            opt_tor_move = spine_range[1]
        else:#Goal is below possible shoulder height
            rospy.loginfo("Goal below spine movement range")
            request.ik_request.pose_stamped.pose.position.z -= spine_range[0]
            opt_tor_move = spine_range[0]
        #Check best possible case
        print "Optimal Torso move: ", opt_tor_move
        ik_goal = self.ik_pose_proxy(request)
        if ik_goal.error_code.val != 1:
            #Return false: Not achievable, even for best case
            rospy.loginfo("Original goal cannot be reached")
            self.log_out.publish(data="Original goal cannot be reached")
            return [False, opt_tor_move]
        opt_ik_goal = ik_goal
        
        #Achievable: Find a reasonable solution, move torso, return true
        rospy.loginfo("Goal can be reached by moving spine")
        self.log_out.publish(data="Goal can be reached by moving spine")
        trials = np.linspace(0,opt_tor_move,round(abs(opt_tor_move)/0.05))
        np.append(trials,opt_tor_move)
        rospy.loginfo("Torso far from best position, finding closer sol.")
        print "Trials: ", trials
        for trial in trials:
            request.ik_request.pose_stamped.pose.position.z = goal_z + trial
            print "Trying goal @ ", goal_z + trial
            ik_goal = self.ik_pose_proxy(request)
            if ik_goal.error_code.val == 1:
                rospy.loginfo("Tried: %s, Succeeded" %trial)
                return [True, trial]
            rospy.loginfo("Tried: %s, Failed" %trial)
        #Broke through somehow, catch error
        return [True, opt_tor_move]
   

    def full_ik_check(self, ps):
        #Check goal as given, if reachable, return true
        req = self.form_ik_request(ps)
        ik_goal = self.ik_pose_proxy(req)
        if ik_goal.error_code.val == 1:
            self.dist = self.calc_dist(self.curr_pose(), ps)
            return (True, ik_goal, None)
        #Check goal with vertical torso movement, if works, return true 
        (torso_succeeded, torso_move) = self.check_torso(req)
        self.move_torso(self.torso_state.actual.positions[0]+torso_move)
        duration = max(4,85*abs(torso_move))
        if torso_succeeded:
            ik_goal = self.ik_pose_proxy(req)
            self.dist = self.calc_dist(self.curr_pose(), ps)
            if ik_goal.error_code.val ==1:
                return (True, ik_goal, duration)
            else:
                print "Reported Succeeded, then really failed: \r\n", ik_goal
        
        #Check goal incrementally retreating hand pose, if works, return true
        pct = 0.98
        curr_pos = self.curr_pose().pose.position
        dx = req.ik_request.pose_stamped.pose.position.x - curr_pos.x
        dy = req.ik_request.pose_stamped.pose.position.y - curr_pos.y
        while pct > 0.01:
            #print "Percent: %s" %pct
            req.ik_request.pose_stamped.pose.position.x = curr_pos.x + pct*dx
            req.ik_request.pose_stamped.pose.position.y = curr_pos.y + pct*dy
            ik_goal = self.ik_pose_proxy(req)
            if ik_goal.error_code.val == 1:
                rospy.loginfo("Succeeded PARTIALLY (%s) with torso" %pct)
                return (True, ik_goal, duration)
            else:
                pct -= 0.02
        #Nothing worked, report failure, return false
        rospy.loginfo("IK Failed: Error Code %s" %str(ik_goal.error_code))
        rospy.loginfo("Reached as far as possible")
        self.log_out.publish(data="Cannot reach farther in this direction.")    
        return (False, ik_goal, duration)

    def form_constraint_aware_ik_request(self, ps):
        #print "forming IK request for :%s" %ps
        req = GetConstraintAwarePositionIKRequest()
        req.timeout = rospy.Duration(5)
        req.ik_request.pose_stamped = ps
        req.ik_request.ik_link_name = \
                    self.ik_info.kinematic_solver_info.link_names[-1]
        req.ik_request.ik_seed_state.joint_state.name = \
                    self.ik_info.kinematic_solver_info.joint_names
        req.ik_request.ik_seed_state.joint_state.position = \
                    self.joint_state_pos
        return req

    def form_ik_request(self, ps):
        #print "forming IK request for :%s" %ps
        req = GetPositionIKRequest()
        req.timeout = rospy.Duration(5)
        req.ik_request.pose_stamped = ps 
        req.ik_request.ik_link_name = \
                    self.ik_info.kinematic_solver_info.link_names[-1]
        req.ik_request.ik_seed_state.joint_state.name = \
                    self.ik_info.kinematic_solver_info.joint_names
        req.ik_request.ik_seed_state.joint_state.position = \
                    self.joint_state_pos
        return req
    
    def send_joint_angles(self, angles, duration = None):
        point = JointTrajectoryPoint()
        point.positions = angles
        self.send_traj_point(point, duration)

    def send_traj_point(self, point, time=None):
        if time is None: 
            point.time_from_start = rospy.Duration(max(20*self.dist, 4))
        else:
            point.time_from_start = rospy.Duration(time)
        joint_traj = JointTrajectory()
        joint_traj.header.stamp = rospy.Time.now()
        # ARD
        # joint_traj.header.frame_id = '/torso_lift_link'
        joint_traj.header.frame_id = 'base_link'
        joint_traj.joint_names = self.ik_info.kinematic_solver_info.joint_names
        joint_traj.points.append(point)
        joint_traj.points[0].velocities = [0,0,0,0,0,0,0]
        arm_goal = JointTrajectoryGoal()
        arm_goal.trajectory = joint_traj
        self.arm_traj_client.send_goal(arm_goal)

class PR2Arm_Planning(PR2Arm):

    def __init__(self, arm, tfl=None):
        self.joint_state_pos = list()
        self.joint_state_vel = list()
        PR2Arm.__init__(self, arm, tfl)
        
        self.move_arm_client = actionlib.SimpleActionClient(
                                     'move_' + self.arm + '_arm', MoveArmAction)
        rospy.loginfo("Waiting for move_" + self.arm + "_arm server")
        if self.move_arm_client.wait_for_server(rospy.Duration(50)):
            rospy.loginfo("Found move_" + self.arm + "_arm server")
        else:
            rospy.logwarn("Cannot find move_" + self.arm + "_arm server")

    def move_arm_to(self, goal_in):
        (reachable, ik_goal, duration) = self.full_ik_check(goal_in)
        rospy.sleep(duration)
        
        rospy.loginfo("Composing move_"+self.arm+"_arm goal")
        goal_out = MoveArmGoal()
        goal_out.motion_plan_request.group_name = self.arm+"_arm"
        goal_out.motion_plan_request.num_planning_attempts = 10 
        goal_out.motion_plan_request.planner_id = ""
        goal_out.planner_service_name = "ompl_planning/plan_kinematic_path"
        goal_out.motion_plan_request.allowed_planning_time = rospy.Duration(1.0)
        
        pos = PositionConstraint()
        pos.header.frame_id = goal_in.header.frame_id 
        pos.link_name = self.arm[0]+"_wrist_roll_link"
        pos.position = goal_in.pose.position

        pos.constraint_region_shape.type = 0 
        pos.constraint_region_shape.dimensions=[0.01]

        pos.constraint_region_orientation = Quaternion(0,0,0,1)
        pos.weight = 1

        goal_out.motion_plan_request.goal_constraints.position_constraints.append(pos)
    
        ort = OrientationConstraint()    
        ort.header.frame_id=goal_in.header.frame_id
        ort.link_name = self.arm[0]+"_wrist_roll_link"
        ort.orientation = goal_in.pose.orientation
        
        ort.absolute_roll_tolerance = 0.02
        ort.absolute_pitch_tolerance = 0.02
        ort.absolute_yaw_tolerance = 0.02
        ort.weight = 0.5
        goal_out.motion_plan_request.goal_constraints.orientation_constraints.append(ort)
        goal_out.accept_partial_plans = True
        goal_out.accept_invalid_goals = True
        goal_out.disable_collision_monitoring = True
        rospy.loginfo("Sending composed move_"+self.arm+"_arm goal")

        finished_within_time = False
        self.move_arm_client.send_goal(goal_out)
        finished_within_time = self.move_arm_client.wait_for_result(
                                                            rospy.Duration(60))
        if not (finished_within_time):
            self.move_arm_client.cancel_goal()
            self.log_out.publish(data="Timed out achieving "+self.arm+
                                         " arm goal pose")
            rospy.loginfo("Timed out achieving right arm goal pose")
            return False
        else:
            state = self.move_arm_client.get_state()
            result = self.move_arm_client.get_result()
            if (state == 3): #3 == SUCCEEDED
                rospy.loginfo("Action Finished: %s" %state)
                self.log_out.publish(data="Move "+self.arm.capitalize()+
                                          " Arm with Motion Planning: %s"
                                          %self.move_arm_error_dict[
                                                result.error_code.val])
                return True
            else:
                if result.error_code.val == 1:
                    rospy.loginfo("Move_"+self.arm+"_arm_action failed: \
                                    Unable to plan a path to goal")
                    self.log_out.publish(data="Move "+self.arm.capitalize()+
                                        " Arm with Motion Planning Failed: \
                                         Unable to plan a path to the goal")
                elif result.error_code.val == -31:
                    (torso_succeeded, pos) = self.check_torso(
                                                self.form_ik_request(goal_in))
                    if torso_succeeded:
                        rospy.loginfo("IK Failed in Current Position. \
                                        Adjusting Height and Retrying")
                        self.log_out.publish(data="IK Failed in Current \
                                                      Position. Adjusting \
                                                      Height and Retrying")
                        self.move_arm_to(pos)
                    else:
                        rospy.loginfo("Move_"+self.arm+"_arm action failed: %s"
                                        %state)
                        rospy.loginfo("Failure Result: %s" %result)
                        self.log_out.publish(data="Move "+self.arm.capitalize()+
                                                    " Arm with Motion Planning \
                                                    and Torso Check Failed: %s"
                                                    %self.move_arm_error_dict[
                                                        result.error_code.val])
                else:
                    rospy.loginfo("Move_"+self.arm+"_arm action failed: %s" %state)
                    rospy.loginfo("Failure Result: %s" %result)
                    self.log_out.publish(data="Move "+self.arm.capitalize()+
                                                " Arm with Motion Planning \
                                                Failed: %s" 
                                                %self.move_arm_error_dict[
                                                        result.error_code.val])
            return False


if __name__ == '__main__':
    rospy.init_node('pr2_arms')
    left_arm = PR2Arm('left')
    right_arm = PR2Arm('right')
    rospy.spin()


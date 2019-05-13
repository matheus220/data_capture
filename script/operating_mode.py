#!/usr/bin/env python

import json
import time
import threading

import rospy
from data_capture.srv import RequestSaveData, SetBatteryLevel
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State, StateMachine, Iterator, Concurrence, loginfo
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32, String
from tf.transformations import quaternion_from_euler


class Setup(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

    def execute(self, userdata):
        # Your state execution goes here
        return 'succeeded'


class ModeManager(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['patrol', 'verification', 'manual', 'standby', 'aborted', 'preempted'],
                       input_keys=['config', 'nav_status'],
                       output_keys=['config', 'nav_status'])

        self._current_mode = None
        self._current_waypoints = []
        self._incomplete_missions = []
        self._patrol_waypoints = []
        self._patrol_pause = 0

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if userdata.nav_status:
            nav_outcome = userdata.nav_status['outcome']
            if nav_outcome != 'succeeded':
                mission = {}
                mission['mode'] = self._current_mode
                mission['start_wp_id'] = userdata.nav_status['last_visited_wp'] + 1
                mission['waypoints'] = self._current_waypoints
                self._incomplete_missions.append(mission)
                userdata.nav_status.clear()

        userdata.config['start_wp_id'] = 0
        userdata.config['incomplete_mission'] = False

        # Your state execution goes here
        if 'change_mode' in userdata.config and userdata.config['change_mode']:
            self._current_mode = userdata.config['mode']
            if self._current_mode == 'manual':
                self._current_waypoints = []
            else:
                self._current_waypoints = userdata.config['waypoints']
            userdata.config['change_mode'] = False
            if self._current_mode == 'patrol':
                self._patrol_waypoints = self._current_waypoints
                self._patrol_pause = userdata.config['pause']
            return self._current_mode
        elif self._current_mode:
            if self._incomplete_missions:
                last_mission = self._incomplete_missions.pop()
                userdata.config['start_wp_id'] = last_mission['start_wp_id']
                userdata.config['incomplete_mission'] = True
                userdata.config['mode'] = last_mission['mode']
                userdata.config['waypoints'] = last_mission['waypoints']
                return last_mission['mode']
            elif self._patrol_waypoints:
                userdata.config['mode'] = 'patrol'
                userdata.config['waypoints'] = self._patrol_waypoints
                userdata.config['pause'] = self._patrol_pause
                return 'patrol'
            else:
                return 'standby'
        else:
            return 'standby'


class PatrolControl(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['navigation', 'pause', 'aborted', 'preempted'],
                       input_keys=['config'])
        self.last_outcomes = None

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if userdata.config['incomplete_mission'] or self.last_outcomes != 'navigation':
            self.last_outcomes = 'navigation'
            return 'navigation'
        else:
            self.last_outcomes = 'pause'
            return 'pause'


class Pause(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'preempted'],
                       input_keys=['config'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        time.sleep(userdata.config['pause'])

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'succeeded'


class Manual(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.WAIT_TIME_SECONDS = 1
        self.pub = rospy.Publisher("/geko/base_controller/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("manual_mode_cmd_vel", Twist, self.manual_cmd_cb)

        self._trigger_event = threading.Event()

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self._trigger_event.clear()

        while not self._trigger_event.wait(self.WAIT_TIME_SECONDS):
            self.check_stop()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

    def check_stop(self):
        if self.sub.get_num_connections() < 1:
            loginfo("Connection with controller lost, leaving 'MANUAL' mode")
            self._trigger_event.set()

    def manual_cmd_cb(self, msg):
        self._time_last_cmd = rospy.Time.now().to_sec()
        self.pub.publish(msg)

    def request_preempt(self):
        State.request_preempt(self)
        self._trigger_event.set()


class Verification(State):
    def __init__(self):
        State.__init__(self, outcomes=['navigation', 'aborted', 'preempted'])

    def execute(self, userdata):
        # Your state execution goes here
        return 'navigation'


class OperatingMode(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                              outcomes=['standby', 'aborted', 'preempted'],
                              input_keys=['config'])

        self.userdata.nav_status = {}

        with self:
            # ===== MODE_MANAGER State =====
            StateMachine.add('MODE_MANAGER',
                             ModeManager(),
                             transitions={'patrol': 'PATROL',
                                          'verification': 'VERIFICATION',
                                          'manual': 'MANUAL',
                                          'standby': 'standby'})

            # ===== PATROL State =====
            StateMachine.add('PATROL',
                             PatrolControl(),
                             transitions={'navigation': 'WAYPOINTS_ITERATOR',
                                          'pause': 'PAUSE'})

            StateMachine.add('PAUSE',
                             Pause(),
                             transitions={'succeeded': 'PATROL'})

            # ===== VERIFICATION State =====
            StateMachine.add('VERIFICATION',
                             Verification(),
                             transitions={'navigation': 'WAYPOINTS_ITERATOR'})

            # ===== MANUAL State =====
            StateMachine.add('MANUAL',
                             Manual(),
                             transitions={'succeeded': 'MODE_MANAGER'})

            # ===== WAYPOINTS_ITERATOR State =====
            self.sm_nav_iterator = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                            input_keys=['config'],
                                            it=[],
                                            output_keys=['config'],
                                            it_label='waypoint_id',
                                            exhausted_outcome='succeeded')

            self.sm_nav_iterator.register_termination_cb(self.save_log_iterator)
            self.sm_nav_iterator.register_start_cb(self.set_it_from_userdata)

            with self.sm_nav_iterator:
                self.sm_nav = StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                      input_keys=['waypoint_id', 'config'])

                with self.sm_nav:
                    StateMachine.add('MOVE_BASE',
                                     SimpleActionState('move_base',
                                                       MoveBaseAction,
                                                       input_keys=['waypoint_id', 'config'],
                                                       goal_cb=self.move_base_goal_cb),
                                     transitions={'succeeded': 'SAVE_DATA'})

                    self.save_data_service = ServiceState('request_save_data',
                                                          RequestSaveData,
                                                          response_cb=self.save_data_response_cb)

                    StateMachine.add('SAVE_DATA',
                                     self.save_data_service,
                                     transitions={'succeeded': 'continue'})

                # Close the sm_nav machine and add it to the iterator
                Iterator.set_contained_state('WAYPOINTS_NAV',
                                             self.sm_nav,
                                             loop_outcomes=['continue'])

            StateMachine.add('WAYPOINTS_ITERATOR',
                             self.sm_nav_iterator,
                             {'succeeded': 'MODE_MANAGER'})

    def save_data_response_cb(self, userdata, response):
        if self.save_data_service.preempt_requested():
            self.save_data_service.service_preempt()
            return 'preempted'
        return 'succeeded'

    def move_base_result_cb(self, userdata, status, result):
        pass

    def move_base_goal_cb(self, userdata, goal):
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        waypoint_id = userdata.waypoint_id
        waypoints = userdata.config['waypoints']
        waypoint = next(wp for wp in waypoints if wp["id"] == waypoint_id)

        position = Point(waypoint['x'], waypoint['y'], 0.0)
        q_angle = quaternion_from_euler(0, 0, waypoint['theta'], axes='sxyz')
        orientation = Quaternion(*q_angle)

        nav_goal.target_pose.pose.position = position
        nav_goal.target_pose.pose.orientation = orientation

        return nav_goal

    def set_it_from_userdata(self, userdata, initial_states):
        with self.sm_nav_iterator:
            wp = userdata.config['waypoints']
            start_id = userdata.config['start_wp_id']
            Iterator.set_iteritems(range(start_id, len(wp)), 'waypoint_id')

    def save_log_iterator(self, userdata, terminal_states, container_outcome):
        if container_outcome == 'continue' or container_outcome == 'succeeded':
            self.userdata.nav_status['last_visited_wp'] = userdata.waypoint_id
        else:
            self.userdata.nav_status['last_visited_wp'] = userdata.waypoint_id - 1

        self.userdata.nav_status['outcome'] = container_outcome


class Main():
    def __init__(self):
        rospy.init_node('operating_mode', anonymous=False)

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # Create the top level SMACH state machine
        self.sm_top = StateMachine(outcomes=['stop'])

        # Open the container
        with self.sm_top:
            # ===== SETUP State =====
            StateMachine.add('SETUP',
                             Setup(),
                             transitions={'succeeded': 'TOP_CONTROL',
                                          'preempted': 'stop',
                                          'aborted': 'stop'})

            # ===== CHARGE State =====
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            pose = Pose()
            pose.position.x = -1.5
            pose.position.y = 4.0
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            nav_goal.target_pose.pose = pose
            nav_docking_station = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal)

            self.sm_recharge = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

            with self.sm_recharge:
                StateMachine.add('NAV_DOCKING_STATION', nav_docking_station,
                                 transitions={'succeeded': 'RECHARGE_BATTERY'})
                StateMachine.add('RECHARGE_BATTERY',
                                 ServiceState('battery_simulator/set_battery_level', SetBatteryLevel, 100),
                                 transitions={'succeeded': 'succeeded'})
            StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded': 'SETUP',
                                                                        'preempted': 'stop',
                                                                        'aborted': 'stop'})

            # ===== TOP_CONTROL State =====
            self.sm_battery_concurrence = Concurrence(outcomes=['restart', 'recharge', 'preempted', 'aborted', 'stop'],
                                                      default_outcome='recharge',
                                                      child_termination_cb=self.child_termination_cb,
                                                      outcome_cb=self.outcome_cb
                                                      )

            self.sm_battery_concurrence.userdata.config_monitor = {'mode': None, 'change_mode': False}

            # Open the container
            with self.sm_battery_concurrence:
                # Add states to the container
                Concurrence.add('BATTERY_MONITOR', MonitorState("/battery_level",
                                                                Float32,
                                                                self.battery_monitor_cb))

                Concurrence.add('CHANGE_MODE_MONITOR',
                                MonitorState("/change_mode",
                                             String,
                                             self.change_mode_monitor_cb,
                                             output_keys=['config_monitor'],
                                             input_keys=['config_monitor']))

                Concurrence.add('OPERATING_MODE',
                                OperatingMode(),
                                remapping={'config': 'config_monitor'})

            StateMachine.add('TOP_CONTROL',
                             self.sm_battery_concurrence,
                             transitions={'restart': 'TOP_CONTROL',
                                          'recharge': 'RECHARGE',
                                          'aborted': 'stop',
                                          'preempted': 'stop'})

        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('mode', self.sm_top, '/SM_ROOT')
        intro_server.start()

        # Execute the state machine
        sm_outcome = self.sm_top.execute()

        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))

        intro_server.stop()

    def child_termination_cb(self, outcome_map):
        if outcome_map['BATTERY_MONITOR'] == 'invalid':
            return True
        elif outcome_map['CHANGE_MODE_MONITOR'] == 'invalid':
            return True
        else:
            return False

    def outcome_cb(self, outcome_map):
        if outcome_map['BATTERY_MONITOR'] == 'invalid':
            return 'recharge'
        elif outcome_map['CHANGE_MODE_MONITOR'] == 'invalid':
            return 'restart'
        else:
            return 'stop'

    def battery_monitor_cb(self, userdata, msg):
        if msg.data < 20.0:
            return False
        else:
            return True

    def change_mode_monitor_cb(self, userdata, msg):
        if msg.data:
            data = json.loads(msg.data)
            if not 'stamp' in userdata.config_monitor.keys() or userdata.config_monitor['stamp'] != data['stamp']:
                userdata.config_monitor = data
                userdata.config_monitor['change_mode'] = True
                return False
        else:
            return True

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        self.sm_top.request_preempt()

        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Main()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")

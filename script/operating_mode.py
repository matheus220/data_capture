#!/usr/bin/env python

import datetime
import json
import threading
import time

import rospy
from data_capture.srv import RequestSaveData, SetBatteryLevel
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from mongo_bridge.msg import Data, DataArray
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
    MODES = ['patrol', 'pause', 'verification', 'manual', 'standby']

    def __init__(self):
        State.__init__(self,
                       outcomes=['patrol', 'pause', 'verification', 'manual', 'standby', 'aborted', 'preempted'],
                       input_keys=['configs', 'cycle_log'],
                       output_keys=['configs', 'cycle_log'])

        self._pub = rospy.Publisher('/mongo_bridge/data', DataArray, queue_size=1)
        self._count_cycles_id = 0
        self._count_id = 0
        self._default_configs = None
        self._current_configs = self._default_configs
        self._incomplete_cycles = []

    def execute(self, userdata):
        # Check if the last cycle was successfully completed
        if userdata.cycle_log:
            nav_outcome = userdata.cycle_log['status']
            if nav_outcome != 'succeeded' and userdata.cycle_log['mode'] in ['verification', 'patrol']:
                mission = self._current_configs.copy()
                mission['start_wp_id'] = userdata.cycle_log['last_visited_wp'] + 1
                mission['cycle_id'] = userdata.cycle_log['cycle_id']
                self._incomplete_cycles.append(mission)
            elif nav_outcome == 'aborted':
                if self._default_configs['stamp'] == self._current_configs['stamp']:
                    self._default_configs = None
            userdata.cycle_log.clear()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        userdata.configs['start_wp_id'] = 0
        mode = ''

        if 'change_mode' in userdata.configs and userdata.configs['change_mode']:
            userdata.configs['change_mode'] = False
            if userdata.configs['mode'] == 'patrol':
                self._default_configs = userdata.configs
        elif self._incomplete_cycles:
            userdata.configs = self._incomplete_cycles.pop()
        elif self._default_configs:
            userdata.configs = self._default_configs
        else:
            userdata.configs.clear()
            mode = 'standby'

        if not mode:
            self._current_configs = userdata.configs
            mode = userdata.configs['mode'] if userdata.configs['mode'] in ModeManager.MODES else 'standby'

        if 'cycle_id' in userdata.configs.keys():
            userdata.cycle_log['cycle_id'] = userdata.configs['cycle_id']
        else:
            userdata.cycle_log['cycle_id'] = self._count_cycles_id
            self._count_cycles_id += 1

        userdata.cycle_log['mode'] = mode
        userdata.cycle_log['status'] = 'active'
        userdata.cycle_log['start_time'] = str(datetime.datetime.now()).split('.')[0]
        self.save_log(userdata.cycle_log)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        print(userdata.configs)
        return mode

    def save_log(self, cycle_log):
        data = Data()
        data.dataModule = 'schemas'
        data.dataClass = 'Cycle'
        data.jsonData = json.dumps(cycle_log)

        data_array = DataArray()
        data_array.data.append(data)

        self._pub.publish(data_array)


class PatrolControl(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['navigation', 'aborted', 'preempted'])
        self.last_outcomes = None

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        else:
            return 'navigation'


class Pause(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'preempted'],
                       input_keys=['configs'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        time.sleep(userdata.configs['pause'])

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
        self._time_last_cmd = 0.0

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
        return 'navigation'


class OperatingMode(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                              outcomes=['succeeded', 'standby', 'aborted', 'preempted'],
                              input_keys=['configs'],
                              output_keys=['cycle_log'])
        self.userdata.cycle_log = {}
        self._pub = rospy.Publisher('/mongo_bridge/data', DataArray, queue_size=1)

        self.register_termination_cb(self.operating_mode_termination_cb)

        with self:
            # ===== MODE_MANAGER State =====
            StateMachine.add('MODE_MANAGER',
                             ModeManager(),
                             transitions={'patrol': 'PATROL',
                                          'verification': 'VERIFICATION',
                                          'manual': 'MANUAL',
                                          'pause': 'PAUSE',
                                          'standby': 'standby'})

            # ===== PATROL State =====
            StateMachine.add('PATROL',
                             PatrolControl(),
                             transitions={'navigation': 'WAYPOINTS_ITERATOR'})

            StateMachine.add('PAUSE',
                             Pause(),
                             transitions={'succeeded': 'succeeded'})

            # ===== VERIFICATION State =====
            StateMachine.add('VERIFICATION',
                             Verification(),
                             transitions={'navigation': 'WAYPOINTS_ITERATOR'})

            # ===== MANUAL State =====
            StateMachine.add('MANUAL',
                             Manual(),
                             transitions={'succeeded': 'succeeded'})

            # ===== WAYPOINTS_ITERATOR State =====
            self.sm_nav_iterator = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                            input_keys=['configs', 'cycle_log'],
                                            it=[],
                                            output_keys=['configs', 'cycle_log'],
                                            it_label='waypoint_id',
                                            exhausted_outcome='succeeded')

            self.sm_nav_iterator.register_start_cb(self.set_it_from_userdata)
            self.sm_nav_iterator.register_termination_cb(self.set_last_visited_wp)

            with self.sm_nav_iterator:
                self.sm_nav = StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                           input_keys=['waypoint_id', 'cycle_log', 'configs'])

                with self.sm_nav:
                    StateMachine.add('MOVE_BASE',
                                     SimpleActionState('move_base',
                                                       MoveBaseAction,
                                                       input_keys=['waypoint_id', 'configs'],
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
                             {'succeeded': 'succeeded'})

    def save_log(self, cycle_log):
        data = Data()
        data.dataModule = 'schemas'
        data.dataClass = 'Cycle'
        data.jsonData = json.dumps(cycle_log)

        data_array = DataArray()
        data_array.data.append(data)

        self._pub.publish(data_array)

    def operating_mode_termination_cb(self, userdata, terminal_states, container_outcome):
        userdata.cycle_log['status'] = container_outcome
        userdata.cycle_log['end_time'] = str(datetime.datetime.now()).split('.')[0]
        self.save_log(userdata.cycle_log)

    def set_last_visited_wp(self, userdata, terminal_states, container_outcome):
        if container_outcome == 'continue' or container_outcome == 'succeeded':
            userdata.cycle_log['last_visited_wp'] = userdata.waypoint_id
        else:
            userdata.cycle_log['last_visited_wp'] = userdata.waypoint_id - 1

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
        waypoints = userdata.configs['waypoints']
        waypoint = next(wp for wp in waypoints if wp["id"] == waypoint_id)

        position = Point(waypoint['x'], waypoint['y'], 0.0)
        q_angle = quaternion_from_euler(0, 0, waypoint['theta'], axes='sxyz')
        orientation = Quaternion(*q_angle)

        nav_goal.target_pose.pose.position = position
        nav_goal.target_pose.pose.orientation = orientation

        return nav_goal

    def set_it_from_userdata(self, userdata, initial_states):
        with self.sm_nav_iterator:
            wp = userdata.configs['waypoints']
            start_id = userdata.configs['start_wp_id']
            Iterator.set_iteritems(range(start_id, len(wp)), 'waypoint_id')


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

            self.sm_battery_concurrence.userdata.mode_configs = {'mode': None, 'change_mode': False}

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
                                             output_keys=['mode_configs'],
                                             input_keys=['mode_configs']))

                self.operating_mode = StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'standby'],
                                                   input_keys=['mode_configs'])

                with self.operating_mode:
                    StateMachine.add('ST_OPERATING_MODE',
                                     OperatingMode(),
                                     transitions={'succeeded': 'ST_OPERATING_MODE'},
                                     remapping={'configs': 'mode_configs'})

                Concurrence.add('OPERATING_MODE',
                                self.operating_mode)

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
        if outcome_map['BATTERY_MONITOR'] == 'invalid' or \
                outcome_map['CHANGE_MODE_MONITOR'] == 'invalid' or \
                outcome_map['OPERATING_MODE'] == 'aborted' or \
                outcome_map['OPERATING_MODE'] == 'preempted':
            return True
        else:
            return False

    def outcome_cb(self, outcome_map):
        if outcome_map['BATTERY_MONITOR'] == 'invalid':
            return 'recharge'
        elif outcome_map['CHANGE_MODE_MONITOR'] == 'invalid' or \
                outcome_map['OPERATING_MODE'] == 'aborted' or \
                outcome_map['OPERATING_MODE'] == 'preempted':
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
            if not hasattr(self, '_last_msg_stamp') or self._last_msg_stamp != data['stamp']:
                userdata.mode_configs = data
                userdata.mode_configs['change_mode'] = True
                self._last_msg_stamp = data['stamp']
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

#!/usr/bin/env python
from mongodb_bridge import MongoBridge

import datetime
import json
import threading
import time

import rospy
from pymongo import MongoClient, DESCENDING as order_des
from bson import ObjectId
from data_capture.srv import RequestSaveData, SetBatteryLevel
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State, StateMachine, Iterator, Concurrence, loginfo, cb_interface, CBState
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32, String, ColorRGBA
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

mb = None
pym = MongoClient().robotic

marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)


def add_marker_rviz(waypoints, color):
    array = MarkerArray()
    for wp in waypoints:
        marker = Marker(type=Marker.ARROW, ns='vis_rviz', action=Marker.ADD)
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = wp[u'id']
        marker.scale.x = 0.35
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.pose.position = Point(wp[u'x'], wp[u'y'], 0.0)
        q_angle = quaternion_from_euler(0, 0, wp[u'theta'] - 1.570796, axes='sxyz')
        marker.pose.orientation = Quaternion(*q_angle)

        if color == 1:
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
        elif color == 2:
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        elif color == 3:
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        elif color == 4:
            marker.color = ColorRGBA(0.4, 1.0, 0.4, 1.0)

        # marker.lifetime = rospy.Duration(600.0)

        array.markers.append(marker)
    marker_publisher.publish(array)


def deleteall_marker_rviz():
    array = MarkerArray()
    marker = Marker(type=Marker.ARROW, ns='vis_rviz', action=Marker.DELETEALL)
    array.markers.append(marker)
    marker_publisher.publish(array)


class Setup(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

    def execute(self, userdata):
        # Your state execution goes here
        return 'succeeded'


class ModeManager(State):
    MODES = ['patrol', 'assistance']

    def __init__(self):
        State.__init__(self,
                       outcomes=['patrol', 'assistance', 'standby', 'aborted', 'preempted'],
                       input_keys=['mode_change_infos', 'mode_infos'],
                       output_keys=['mode_infos'])

    def execute(self, ud):

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ud.mode_infos = {}
        ud.mode_infos["continue_last_mission"] = False
        print(ud.mode_change_infos)

        if ud.mode_change_infos.has_key('change_mode') \
                and ud.mode_change_infos['change_mode'] == True \
                and ud.mode_change_infos['mode'] not in ModeManager.MODES:
            ud.mode_change_infos['change_mode'] = False
            if ud.mode_change_infos['mode'] == 'patrol':
                ud.mode_infos['patrol_id'] = ud.mode_change_infos['patrol_id']
            return ud.mode_change_infos['mode']
        else:
            try:
                id_patrol_default = pym.item.find_one({'name': 'mode'},
                                                      {'params.default_patrol': 1})['params']['default_patrol']
                ud.mode_infos['patrol_id'] = id_patrol_default
                ud.mode_infos['continue_last_mission'] = True
                return 'patrol'
            except (TypeError, KeyError):
                return 'standby'


class Recharge(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                              outcomes=['succeeded', 'aborted', 'preempted'])

        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        pose = Pose()
        pose.position = Point(8.2, -3.0, 0.0)
        pose.orientation = Quaternion(0.0, 0.0, 1.0, 0.0)
        nav_goal.target_pose.pose = pose
        self.nav_docking_station = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal)

        with self:
            StateMachine.add('NAV_DOCKING_STATION', self.nav_docking_station,
                             transitions={'succeeded': 'RECHARGE_BATTERY',
                                          'aborted': 'NAV_DOCKING_STATION'})

            StateMachine.add('RECHARGE_BATTERY',
                             ServiceState('battery_simulator/set_battery_level', SetBatteryLevel, 100),
                             transitions={'succeeded': 'succeeded'})


class Waiting(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'preempted'],
                       input_keys=['log_mission'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        thread = threading.Thread(target=self.waiting(userdata.log_mission['start_time'], userdata.log_mission['patrol']['min_duration']))

        thread.start()

        thread.join()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

    def waiting(self, start_time, duration):
        while True:
            if self.preempt_requested() or ( datetime.datetime.now() > (start_time + datetime.timedelta(seconds=duration)) ):
                break


class Pause(Concurrence):
    def __init__(self):
        Concurrence.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             default_outcome='succeeded',
                             input_keys=['log_mission'],
                             output_keys=['log_mission'],
                             child_termination_cb=self.child_termination_cb,
                             outcome_map={'succeeded': {'WAITING': 'succeeded'}})

        self._min_time_to_go_recharge = 15.0

        @cb_interface(input_keys=['log_mission'],
                      outcomes=['succeeded', 'recharge'])
        def check_waiting_time(ud):
            navigation_duration = datetime.datetime.now() - ud.log_mission['start_time']
            if ud.log_mission['patrol']['min_duration'] - navigation_duration > self._min_time_to_go_recharge:
                return 'recharge'
            else:
                return 'succeeded'

        self.recharge = StateMachine(input_keys=['log_mission'],
                                     output_keys=['log_mission'],
                                     outcomes=['succeeded', 'aborted', 'preempted'])

        with self.recharge:
            StateMachine.add('CHECK_WAITING_TIME', CBState(check_waiting_time),
                             {'recharge': 'RECHARGE'})

            StateMachine.add('RECHARGE', Recharge())

        with self:
            Concurrence.add('WAITING', Waiting())
            Concurrence.add('CONDITIONAL_RECHARGE', self.recharge)

    def child_termination_cb(self, outcome_map):
        if outcome_map['WAITING'] == 'succeeded':
            return True
        else:
            return False


class PatrolManager(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['navigation', 'pause', 'aborted', 'preempted'],
                       input_keys=['log_mission', 'mode_infos'],
                       output_keys=['log_mission'])

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ud.log_mission.clear()

        try:
            ud.mode_infos["patrol_id"] = ObjectId(ud.mode_infos["patrol_id"])
            patrol_missions = pym.mode.find_one(ud.mode_infos["patrol_id"],
                                                {"data.cycles": 1})["data"]["cycles"]
        except TypeError:
            return 'aborted'

        index_waypoint, index_mission = 0, 0

        if ud.mode_infos["continue_last_mission"]:
            try:
                last_mission = pym.cycle.find_one({"data.patrol.patrol_id": ud.mode_infos["patrol_id"]},
                                                  {"data": 1},
                                                  sort=[("data.timestamp", order_des)])["data"]
                index_mission = last_mission["patrol"]["index_mission"]
                index_waypoint = (last_mission["patrol"]["index_waypoint"] + 1) % len(index_mission)
                if index_waypoint == 0:
                    index_mission = (index_mission + 1) % len(patrol_missions)

                ud.log_mission["cycle_id"] = last_mission["cycle_id"]

                deleteall_marker_rviz()
                add_marker_rviz(patrol_missions[index_mission]['waypoints'][index_waypoint:], 1)
                add_marker_rviz(patrol_missions[index_mission]['waypoints'][0:index_waypoint], 2)
            except TypeError:
                pass
        else:
            try:
                ud.log_mission["cycle_id"] = pym.cycle.find_one({},
                                                                {"data.cycle_id": 1},
                                                                sort=[("data.timestamp", order_des)])["data"]["cycle_id"] + 1
            except TypeError:
                ud.log_mission["cycle_id"] = 0

        mission = patrol_missions[index_mission]

        ud.log_mission["mode"] = 'patrol'
        ud.log_mission["start_time"] = datetime.datetime.now()
        ud.log_mission["status"] = 'active'
        ud.log_mission["patrol"] = {}
        ud.log_mission["patrol"]["patrol_id"] = ud.mode_infos["patrol_id"]
        ud.log_mission["patrol"]["index_waypoint"] = index_waypoint
        ud.log_mission["patrol"]["index_mission"] = index_mission
        ud.log_mission["patrol"]["min_duration"] = mission["min_duration"]

        try:
            ud.log_mission["waypoints"] = mission["waypoints"]
            return 'navigation'
        except KeyError:
            return 'pause'
        finally:
            mb.add_data({'class_name': 'cycle',
                         'item_name': 'cycle',
                         'data': ud.log_mission})
            mb.send_data_and_wait()
            ud.log_mission["_id"] = pym.cycle.find_one({},
                                                       {"_id": 1},
                                                       sort=[("data.timestamp", order_des)])["_id"]


class Navigation(Iterator):
    def __init__(self):
        Iterator.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                input_keys=['log_mission'],
                                it=[],
                                output_keys=['log_mission'],
                                it_label='waypoint_id',
                                exhausted_outcome='succeeded')


        self.register_start_cb(self.set_iterator_list)
        self.register_termination_cb(self.termination_iterator)

        with self:
            self.sm_nav = StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                       input_keys=['waypoint_id', 'log_mission'],
                                       output_keys=['log_mission'])

            self.sm_nav.register_start_cb(self.start_cb_nav)
            self.sm_nav.register_termination_cb(self.termination_cb_nav)

            with self.sm_nav:
                StateMachine.add('MOVE_BASE',
                                 SimpleActionState('move_base',
                                                   MoveBaseAction,
                                                   input_keys=['waypoint_id', 'log_mission'],
                                                   goal_cb=self.move_base_goal_cb),
                                 transitions={'succeeded': 'SAVE_DATA',
                                              'aborted': 'continue'})

                self.save_data_service = ServiceState('make_data_acquisition',
                                                      RequestSaveData,
                                                      response_cb=self.save_data_response_cb)

                StateMachine.add('SAVE_DATA',
                                 self.save_data_service,
                                 transitions={'succeeded': 'continue'})

            # Close the sm_nav machine and add it to the iterator
            Iterator.set_contained_state('WAYPOINTS_NAV',
                                         self.sm_nav,
                                         loop_outcomes=['continue'])

    def set_iterator_list(self, userdata, initial_states):
        with self:
            waypoints = userdata.log_mission['waypoints']
            start_index = userdata.log_mission['patrol']['index_waypoint']
            Iterator.set_iteritems(range(start_index, len(waypoints)), 'waypoint_id')

    def termination_iterator(self, userdata, terminal_states, container_outcome):
        pass
        '''
        userdata.configs['indexes'][1] = userdata.waypoint_id
        if container_outcome != 'succeeded':
            userdata.configs['indexes'][1] -= 1

        next_waypoint_id = (userdata.configs['indexes'][1] + 1) % len(userdata.cycle['waypoints'])
        if next_waypoint_id == 0:
            next_cycle_id = (userdata.configs['indexes'][0] + 1) % len(userdata.configs['cycles'])
        else:
            next_cycle_id = userdata.cycle['patrol_index']

        mb.add_data({'class_name': 'mode',
                     'item_name': 'mode',
                     'params': {'indexes': [next_cycle_id, next_waypoint_id]}})
        mb.send_data_and_wait()
        '''

    def save_data_response_cb(self, userdata, response):
        if self.save_data_service.preempt_requested():
            self.save_data_service.service_preempt()
            return 'preempted'
        return 'succeeded'

    def move_base_goal_cb(self, ud, goal):
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        waypoint = next(wp for wp in ud.log_mission['waypoints'] if wp["id"] == ud.waypoint_id)

        position = Point(waypoint['x'], waypoint['y'], 0.0)
        q_angle = quaternion_from_euler(0, 0, waypoint['theta'], axes='sxyz')
        orientation = Quaternion(*q_angle)

        nav_goal.target_pose.pose.position = position
        nav_goal.target_pose.pose.orientation = orientation

        return nav_goal

    def start_cb_nav(self, userdata, initial_states):
        add_marker_rviz([userdata.log_mission['waypoints'][userdata.waypoint_id]], 4)

    def termination_cb_nav(self, userdata, terminal_states, container_outcome):
        status = None
        if container_outcome == 'continue' or container_outcome == 'succeeded':
            if terminal_states == ['MOVE_BASE']:
                status = 'aborted'
                add_marker_rviz([userdata.log_mission['waypoints'][userdata.waypoint_id]], 3)
            else:
                status = 'succeeded'
                add_marker_rviz([userdata.log_mission['waypoints'][userdata.waypoint_id]], 2)
        else:
            deleteall_marker_rviz()

        pym.cycle.update_one({"_id": userdata.log_mission['_id']},
                             {
                                "$set": {
                                    "data.patrol.index_waypoint": userdata.waypoint_id
                                },
                                "$push": {
                                    "data.navigation": status
                                }
                             }
                            )

class Patrol(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                              outcomes=['succeeded', 'aborted', 'preempted', 'continue'],
                              input_keys=['mode_infos'],
                              output_keys=['log_mission'])

        self.userdata.log_mission = {}

        self.register_termination_cb(self.save_mission_msg)

        with self:
            StateMachine.add('PATROL_MANAGER', PatrolManager(),
                             transitions={'navigation': 'NAVIGATION',
                                          'pause': 'PAUSE'})

            StateMachine.add('NAVIGATION', Navigation(),
                             transitions={'succeeded': 'PAUSE'})

            StateMachine.add('PAUSE', Pause(),
                             transitions={'succeeded': 'continue'})

    def save_mission_msg(self, userdata, terminal_states, container_outcome):
        userdata.cycle_log['status'] = container_outcome
        userdata.cycle_log['end_time'] = str(datetime.datetime.now()).split('.')[0]

        mb.add_data({'class_name': 'cycle',
                     'item_name': 'cycle',
                     'data': userdata.cycle_log})
        mb.send_data_and_wait()


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


class OperatingMode(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                              outcomes=['succeeded', 'standby', 'aborted', 'preempted'],
                              input_keys=['mode_change_infos'],
                              output_keys=['mode_infos'])

        self.userdata.mode_infos = {}

        with self:
            # ===== MODE_MANAGER State =====
            StateMachine.add('MODE_MANAGER',
                             ModeManager(),
                             transitions={'patrol': 'PATROL',
                                          'assistance': 'ASSISTANCE',
                                          'standby': 'standby'})

            # ===== PATROL State =====
            StateMachine.add('PATROL',
                             Patrol(),
                             transitions={'succeeded': 'succeeded',
                                          'continue': 'PATROL'})

            # ===== MANUAL State =====
            StateMachine.add('ASSISTANCE',
                             Manual(),
                             transitions={'succeeded': 'succeeded'})


class Main():
    def __init__(self):
        rospy.init_node('operating_mode', anonymous=False)

        global mb
        mb = MongoBridge()

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

            StateMachine.add('RECHARGE', Recharge(), transitions={'succeeded': 'SETUP',
                                                                  'preempted': 'stop',
                                                                  'aborted': 'stop'})

            # ===== TOP_CONTROL State =====
            self.sm_battery_concurrence = Concurrence(outcomes=['restart', 'recharge', 'preempted', 'aborted', 'stop'],
                                                      default_outcome='recharge',
                                                      child_termination_cb=self.child_termination_cb,
                                                      outcome_cb=self.outcome_cb
                                                      )

            self.sm_battery_concurrence.userdata.mode_change_infos = {'mode': None, 'change_mode': False}

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
                                             input_keys=['mode_change_infos'],
                                             output_keys=['mode_change_infos']))

                self.operating_mode = StateMachine(outcomes=['aborted', 'preempted', 'standby'],
                                                   input_keys=['mode_change_infos'])

                with self.operating_mode:
                    StateMachine.add('ST_OPERATING_MODE',
                                     OperatingMode(),
                                     transitions={'succeeded': 'ST_OPERATING_MODE'})

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
                mb.add_data({'class_name': 'mode',
                           'item_name': 'mode',
                           'data': data})
                mb.send_data_and_wait()
                mode_id = str(pym.mode.find_one({}, {"_id": 1}, sort=[("data.timestamp", order_des)])["_id"])
                if data['default_mode'] == True:
                    mb.add_data({'class_name': 'mode',
                                 'item_name': 'mode',
                                 'params': {'default_patrol': mode_id}})
                    mb.send_data_and_wait()
                userdata.mode_change_infos['mode'] = data['mode']
                userdata.mode_change_infos['mode_id'] = mode_id
                userdata.mode_change_infos['change_mode'] = True
                print('mode_change_infos', userdata.mode_change_infos)
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

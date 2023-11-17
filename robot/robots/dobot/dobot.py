#CODE based on https://github.com/Dobot-Arm/TCP-IP-CR-Python

from time import sleep
import time
import numpy as np
from threading import Thread
import robot.constants as const
from robot.constants import MotionType
import robot.control.robot_processing as robot_process
from robot.robots.dobot.dobot_connection import DobotApiDashboard, DobotApiMove, DobotConnection


class Dobot:
    """
    The class for communicating with Dobot robot.
    """
    DASHBOARD_PORT = 29999
    MOVEMENT_PORT = 30003
    FEEDBACK_PORT = 30004
    TOOL_ID = 0
    TIMEOUT_START_MOTION = 10
    TIMEOUT_MOTION = 45
    ERROR_STATUS = 9

    def __init__(self, ip, robot_config):
        self.ip = ip
        self.robot_config = robot_config

        self.client_dashboard = None
        self.client_feedback = None
        self.client_movement = None

        self.stop_threads = False

        self.moving = False
        self.thread_move = False

        self.coordinates = [None]*6
        self.force_torque_data = [None] * 6
        self.robot_mode = 0
        self.running_status = 0

        self.status_move = False
        self.target = [None] * 6
        self.target_reached = False
        self.motion_type = MotionType.NORMAL

        self.connected = False

    def Connect(self):
        if self.connected:
            self.client_dashboard = None
            self.client_feedback = None
            self.client_movement = None
        try:
            self.client_dashboard = DobotApiDashboard(self.ip, self.DASHBOARD_PORT)
            self.client_movement = DobotApiMove(self.ip, self.MOVEMENT_PORT)
            self.client_feedback = DobotConnection(self.ip, self.FEEDBACK_PORT)
            self.moving = False

            self._set_feedback()
            self._set_move_thread()

            sleep(2)
            if any(coord is None for coord in self.coordinates):
                print("Please, restart robot")
                return

            if self.robot_mode == 4:
                self.client_dashboard.EnableRobot()
                sleep(1)

            if self.robot_mode == 9:
                self.client_dashboard.ClearError()
                sleep(1)

            self.connected = True

        except Exception as e:
            print("Attention!", f"Connection Error:{e}")

    def IsConnected(self):
        return self.connected

    def GetCoordinates(self):
        return self.coordinates

    def SetTargetReached(self, target_reached):
        self.target_reached = target_reached

    # TODO: Note that MoveLinear, MoveCircular, and
    #   TuneRobot functions are almost identical at this stage.
    #   This is because the distinction between the motion types
    #   should really go deeper into the structure of this class,
    #   because the different motion types have different structure and
    #   meaning for the parameter 'target'. Hence, we should not use the same
    #   variable (self.target) for each. After that change is implemented,
    #   the functions below will branch off.

    def MoveLinear(self, linear_target):
        self.target = linear_target
        self.motion_type = motion_type
        self.status_move = True
        if not self.moving:
            self.moving = True
            self._set_move_thread()

    def MoveCircular(self, start_position, waypoint, target):
        # TODO: Start position, waypoint, and target should be stored in three separate
        #   variables, not in one variable (self.target).
        self.target = start_position, waypoint, target
        self.motion_type = motion_type
        self.status_move = True
        if not self.moving:
            self.moving = True
            self._set_move_thread()

    def TuneRobot(self, tuning_target):
        self.target = tuning_target
        self.motion_type = motion_type
        self.status_move = True
        if not self.moving:
            self.moving = True
            self._set_move_thread()

    def ReadForceSensor(self):
        if const.FORCE_TORQUE_SENSOR:
            return self.force_torque_data
        else:
            return False

    def CompensateForce(self):
        status = self.client_dashboard.RobotMode()
        print("CompensateForce")
        if status != self.ERROR_STATUS:
            #self.cobot.SetOverride(0.1)  # Setting robot's movement speed
            else:
                offset_x = 0
                offset_y = 0
                offset_z = -2
                offset_rx = 0
                offset_ry = 0
                offset_rz = 0
                self.client_movement.RelMovLTool(
                  offset_x,
                  offset_y,
                  offset_z,
                  offset_rx,
                  offset_ry,
                  offset_rz,
                  tool=self.TOOL_ID
                )

    def StopRobot(self):
        # Takes some microseconds to the robot actual stops after the command.
        # The sleep time is required to guarantee the stop
        self.status_move = False
        #if self.running_status == 1:
        self.client_dashboard.ResetRobot()
        #sleep(0.05)

    def ForceStopRobot(self):
        self.StopRobot()
        if self.moving:
            self.moving = False
            print("ForceStopRobot")
            if self.thread_move:
                try:
                    self.thread_move.join()
                except RuntimeError:
                    pass

    def Close(self):
        self.StopRobot()
        self.connected = False
        self.moving = False
        if self.thread_move:
            try:
                self.thread_move.join()
            except RuntimeError:
                pass
        #TODO: robot function to close? self.cobot.close()

    ## Internal methods

    def _set_feedback(self):
        if self.connected:
            thread = Thread(target=self._feedback)
            thread.setDaemon(True)
            thread.start()

    def _feedback(self):
        hasRead = 0
        while True:
            if not self.connected:
                break
            data = bytes()
            while hasRead < 1440:
                temp = self.client_feedback.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0

            a = np.frombuffer(data, dtype=MyType)

            # Refresh coordinate points
            self.coordinates = a["tool_vector_actual"][0]
            self.force_torque_data = a["six_force_value"][0]
            #OR self.force_torque_data = a["actual_TCP_force"][0]
            self.robot_mode = int(a["robot_mode"][0])
            self.running_status = int(a["running_status"][0])

            #sleep(0.001)

    def _set_move_thread(self):
        if self.connected:
            thread = Thread(target=self._move_thread)
            thread.daemon = True
            thread.start()
            self.thread_move = thread

    def _motion_loop(self):
        timeout_start = time.time()
        while self.running_status != 1:
            if time.time() > timeout_start + self.TIMEOUT_START_MOTION:
                print("break")
                self.StopRobot()
                break
            sleep(0.001)

        while self.running_status == 1:
            status = int(self.robot_mode)
            if status == self.ERROR_STATUS:
                self.StopRobot()
            if time.time() > timeout_start + self.TIMEOUT_MOTION:
                self.StopRobot()
                print("break")
                break
            sleep(0.001)

    def _move_thread(self):
        while True:
            if not self.moving:
                self.StopRobot()
                break
            if self.status_move and not self.target_reached and not self.running_status:
                print('moving')
                if self.motion_type == MotionType.NORMAL or self.motion_type == MotionType.LINEAR_OUT:
                    self.client_movement.MoveLinear(self.target)
                    self._motion_loop()
                elif self.motion_type == MotionType.ARC:
                    arc_bezier_curve_step = self.robot_config['arc_bezier_curve_step']
                    curve_set = robot_process.bezier_curve(
                        points=np.asarray(self.target),
                        step=arc_bezier_curve_step,
                    )
                    target = self.target
                    for curve_point in curve_set:
                        self.client_movement.ServoP(curve_point)
                        self._motion_loop()
                        if self.motion_type != MotionType.ARC:
                            self.StopRobot()
                            break

                        arc_threshold_distance = self.robot_config['arc_threshold_distance']
                        if not np.allclose(np.array(self.target[2][:3]), np.array(target[2][:3]), 0, arc_threshold_distance):
                            self.StopRobot()
                            break
                        if not self.moving:
                            self.StopRobot()
                            break
                elif self.motion_type == MotionType.TUNING:
                    offset_x = self.target[0]
                    offset_y = self.target[1]
                    offset_z = self.target[2]
                    offset_rx = self.target[3]
                    offset_ry = self.target[4]
                    offset_rz = self.target[5]
                    self.client_movement.RelMovLTool(
                      offset_x,
                      offset_y,
                      offset_z,
                      offset_rx,
                      offset_ry,
                      offset_rz,
                      tool=self.TOOL_ID
                    )
                    self._motion_loop()

            sleep(0.001)

# Port Feedback
MyType = np.dtype([(
    'len',
    np.int64,
), (
    'digital_input_bits',
    np.uint64,
), (
    'digital_output_bits',
    np.uint64,
), (
    'robot_mode',
    np.uint64,
), (
    'time_stamp',
    np.uint64,
), (
    'time_stamp_reserve_bit',
    np.uint64,
), (
    'test_value',
    np.uint64,
), (
    'test_value_keep_bit',
    np.float64,
), (
    'speed_scaling',
    np.float64,
), (
    'linear_momentum_norm',
    np.float64,
), (
    'v_main',
    np.float64,
), (
    'v_robot',
    np.float64,
), (
    'i_robot',
    np.float64,
), (
    'i_robot_keep_bit1',
    np.float64,
), (
    'i_robot_keep_bit2',
    np.float64,
), ('tool_accelerometer_values', np.float64, (3, )),
    ('elbow_position', np.float64, (3, )),
    ('elbow_velocity', np.float64, (3, )),
    ('q_target', np.float64, (6, )),
    ('qd_target', np.float64, (6, )),
    ('qdd_target', np.float64, (6, )),
    ('i_target', np.float64, (6, )),
    ('m_target', np.float64, (6, )),
    ('q_actual', np.float64, (6, )),
    ('qd_actual', np.float64, (6, )),
    ('i_actual', np.float64, (6, )),
    ('actual_TCP_force', np.float64, (6, )),
    ('tool_vector_actual', np.float64, (6, )),
    ('TCP_speed_actual', np.float64, (6, )),
    ('TCP_force', np.float64, (6, )),
    ('Tool_vector_target', np.float64, (6, )),
    ('TCP_speed_target', np.float64, (6, )),
    ('motor_temperatures', np.float64, (6, )),
    ('joint_modes', np.float64, (6, )),
    ('v_actual', np.float64, (6, )),
    # ('dummy', np.float64, (9, 6))])
    ('hand_type', np.byte, (4, )),
    ('user', np.byte,),
    ('tool', np.byte,),
    ('run_queued_cmd', np.byte,),
    ('pause_cmd_flag', np.byte,),
    ('velocity_ratio', np.byte,),
    ('acceleration_ratio', np.byte,),
    ('jerk_ratio', np.byte,),
    ('xyz_velocity_ratio', np.byte,),
    ('r_velocity_ratio', np.byte,),
    ('xyz_acceleration_ratio', np.byte,),
    ('r_acceleration_ratio', np.byte,),
    ('xyz_jerk_ratio', np.byte,),
    ('r_jerk_ratio', np.byte,),
    ('brake_status', np.byte,),
    ('enable_status', np.byte,),
    ('drag_status', np.byte,),
    ('running_status', np.byte,),
    ('error_status', np.byte,),
    ('jog_status', np.byte,),
    ('robot_type', np.byte,),
    ('drag_button_signal', np.byte,),
    ('enable_button_signal', np.byte,),
    ('record_button_signal', np.byte,),
    ('reappear_button_signal', np.byte,),
    ('jaw_button_signal', np.byte,),
    ('six_force_online', np.byte,),
    ('reserve2', np.byte, (82, )),
    ('m_actual', np.float64, (6, )),
    ('load', np.float64,),
    ('center_x', np.float64,),
    ('center_y', np.float64,),
    ('center_z', np.float64,),
    ('user[6]', np.float64, (6, )),
    ('tool[6]', np.float64, (6, )),
    ('trace_index', np.float64,),
    ('six_force_value', np.float64, (6, )),
    ('target_quaternion', np.float64, (4, )),
    ('actual_quaternion', np.float64, (4, )),
    ('reserve3', np.byte, (24, ))])

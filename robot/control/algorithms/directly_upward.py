from enum import Enum

import numpy as np


class MotionSequenceState(Enum):
    NOT_INITIATED = 0
    MOVE_UPWARD = 1
    MOVE_AND_ROTATE_IN_XY_PLANE = 2
    # XXX: Due to calibration imprecision, there is inaccuracy in the estimated displacement; the inaccuracy is larger the farther
    # away the robot is from the target. Therefore, move first partway downward to get a better estimate of the displacement, then rest of
    # the way to the target. If this part-way movement is not first done, the robot will either undershoot, causing the motion sequence to
    # be re-triggered after the previous motion sequence is finished, or overshoot, causing the robot to potentially collide with the head.
    MOVE_PARTWAY_DOWNWARD = 3
    MOVE_TO_TARGET = 4
    FINISHED = 5

    def next(self):
        """Returns the next state in the sequence, and remains in the last state."""
        members = list(MotionSequenceState)
        index = members.index(self)

        # Check if current state is the last state
        if index == len(members) - 1:
            return self

        # Otherwise, proceed to the next state
        return members[index + 1]


class DirectlyUpwardAlgorithm:
    # Thresholds outside which a motion sequence is initiated.
    TRANSLATION_THRESHOLD = 10.0  # mm
    ROTATION_THRESHOLD = 5.0  # degrees

    # The threshold for both distance (in mm) and angle (in degrees) to move to the next axis
    # when performing tuning motion.
    DISTANCE_ANGLE_THRESHOLD = 1.0

    # The proportion of the remaining distance to the target to move partway downward.
    PARTWAY_DOWNWARD_REMAINING_PROPORTION = 0.1

    def __init__(self, robot, config, robot_config):
        self.robot = robot

        self.safe_height = config['safe_height']

        # Unused for now.
        self.robot_config = robot_config

        self.reset_state()

    def reset_state(self):
        self.motion_sequence_state = MotionSequenceState.NOT_INITIATED

    def move_decision(self,
                      displacement_to_target,
                      target_pose_in_robot_space_estimated_from_head_pose,
                      target_pose_in_robot_space_estimated_from_displacement,
                      robot_pose,
                      head_center):

        # If motion sequence is not initiated, check if it should be.
        if self.motion_sequence_state == MotionSequenceState.NOT_INITIATED:

            # Compute the maximum translation and rotation to the target.
            max_translation = np.max(np.abs(displacement_to_target[:3]))
            max_rotation = np.max(np.abs(displacement_to_target[3:]))

            # If the maximum translation or rotation to the target is larger than the threshold, initiate the motion sequence.
            if max_translation > self.TRANSLATION_THRESHOLD or max_rotation > self.ROTATION_THRESHOLD:
                print("Max translation: {:.2f} mm, max rotation: {:.2f} degrees, exceeding the threshold".format(max_translation, max_rotation))
                print("Initiating motion sequence")

                # Start the motion sequence by moving upward.
                self.motion_sequence_state = MotionSequenceState.MOVE_UPWARD

        # If motion sequence is initiated, continue the sequence, otherwise perform tuning motion.
        if self.motion_sequence_state != MotionSequenceState.NOT_INITIATED:
            success = self._perform_motion(target_pose_in_robot_space_estimated_from_displacement)
        else:
            success = self._tune(target_pose_in_robot_space_estimated_from_displacement)

        # If the motion sequence is finished, reset the state.
        if self.motion_sequence_state == MotionSequenceState.FINISHED:
            print("Motion sequence finished")
            self.reset_state()

        # TODO: The force sensor is not normalized for now - add some logic here.
        normalize_force_sensor = False

        return success, normalize_force_sensor

    def _tune(self, target_pose_in_robot_space):
        print("Initiating tuning motion")

        success = self.robot.move_linear(target_pose_in_robot_space)
        return success

    def _perform_motion(self, target_pose_in_robot_space):
        success = False

        if self.motion_sequence_state == MotionSequenceState.MOVE_UPWARD:
            success = self._move_to_safe_height()

        elif self.motion_sequence_state == MotionSequenceState.MOVE_AND_ROTATE_IN_XY_PLANE:
            print("Moving and rotating in XY plane")

            pose = target_pose_in_robot_space
            pose[2] = self.safe_height
            success = self.robot.move_linear(pose)

        elif self.motion_sequence_state == MotionSequenceState.MOVE_PARTWAY_DOWNWARD:
            print("Moving partway downward")

            pose = target_pose_in_robot_space
            pose[2] = pose[2] + self.PARTWAY_DOWNWARD_REMAINING_PROPORTION * (self.safe_height - pose[2])
            success = self.robot.move_linear(pose)

        elif self.motion_sequence_state == MotionSequenceState.MOVE_TO_TARGET:
            print("Moving to target")

            pose = target_pose_in_robot_space
            success = self.robot.move_linear(pose)

        # Transition to the next state if movement command was given successfully.
        if success:
            self.motion_sequence_state = self.motion_sequence_state.next()

        return success

    def move_away_from_head(self):
        print("Moving away from head")

        success = self._move_to_safe_height()
        return success

    def _move_to_safe_height(self):
        print("Moving upward to a safe height")

        success, pose = self.robot.get_pose()
        if not success:
            return False

        pose[2] = self.safe_height
        success = self.robot.move_linear(pose)

        return success

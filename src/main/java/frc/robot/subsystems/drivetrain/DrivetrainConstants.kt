package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.signals.InvertedValue
import frc.robot.EditingConstants

object DrivetrainConstants {
    const val MAX_DRIVE_SPEED = EditingConstants.MAX_DRIVE_SPEED
    const val MAX_TURN_SPEED = EditingConstants.MAX_TURN_SPEED

    const val SUPPLY_CURRENT_LIMIT = 40.0
    const val STATOR_CURRENT_LIMIT = 100.0

    const val SLEW_RATE_LIMIT = 3.0

    const val DRIVETRAIN_OUTPUT_MULTIPLY = 1.0

    const val LEFT_LEADER_ID = -1
    const val LEFT_FOLLOWER_ID = -1
    const val RIGHT_LEADER_ID = -1
    const val RIGHT_FOLLOWER_ID = -1

    val leftInvertedState = InvertedValue.Clockwise_Positive // May need to be swapped
    val rightInvertedState = InvertedValue.CounterClockwise_Positive // May need to be swapped

    const val TRACK_WIDTH = 0.0
    const val WHEEL_RADIUS = 0.0

    const val ODOMETRY_PERIOD = 20
    const val ODOMETRY_FREQUENCY = 1000 / ODOMETRY_PERIOD

}
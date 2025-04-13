package frc.robot.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX

class DrivetrainIOVictorSPX: DrivetrainIO {

    private val leftLeader = WPI_VictorSPX(DrivetrainConstants.LEFT_LEADER_ID)
    private val leftFollower = WPI_VictorSPX(DrivetrainConstants.LEFT_FOLLOWER_ID)
    private val rightLeader = WPI_VictorSPX(DrivetrainConstants.RIGHT_LEADER_ID)
    private val rightFollower = WPI_VictorSPX(DrivetrainConstants.RIGHT_FOLLOWER_ID)

    init {
        /** Motor Configuration **/
        // Left Follower
        leftFollower.setInverted(InvertType.FollowMaster)
        leftFollower.follow(leftLeader)

        // Right Follower
        rightFollower.setInverted(InvertType.FollowMaster)
        rightFollower.follow(rightLeader)

        // Left Leader
        leftLeader.inverted = DrivetrainConstants.LEFT_INVERTED

        // Right Leader
        rightLeader.inverted = DrivetrainConstants.LEFT_INVERTED
    }

    override fun updateInputs(inputs: DrivetrainIO.DrivetrainIOInputs) {
        inputs.leftLeaderAppliedVolts = leftLeader.motorOutputVoltage
        inputs.leftFollowerAppliedVolts = leftFollower.motorOutputVoltage
        inputs.rightLeaderAppliedVolts = rightLeader.motorOutputVoltage
        inputs.rightFollowerAppliedVolts = rightFollower.motorOutputVoltage
    }

    override fun setSpeed(leftSpeed: Double, rightSpeed: Double) {
        leftLeader.set(leftSpeed)
        rightLeader.set(rightSpeed)
    }

    override fun setVoltage(leftVolts: Double, rightVolts: Double) {
        leftLeader.setVoltage(leftVolts)
        rightLeader.setVoltage(rightVolts)
    }

    override fun stop() {
        leftLeader.stopMotor()
        rightLeader.stopMotor()
    }
}
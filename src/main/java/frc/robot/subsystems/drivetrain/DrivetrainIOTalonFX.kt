package frc.robot.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue

class DrivetrainIOTalonFX: DrivetrainIO {

    private val leftLeader = TalonFX(DrivetrainConstants.LEFT_LEADER_ID)
    private val leftFollower = TalonFX(DrivetrainConstants.LEFT_FOLLOWER_ID)
    private val rightLeader = TalonFX(DrivetrainConstants.RIGHT_LEADER_ID)
    private val rightFollower = TalonFX(DrivetrainConstants.RIGHT_FOLLOWER_ID)

    private var leftConfig = TalonFXConfiguration()
    private var rightConfig = TalonFXConfiguration()

    init {

        leftConfig.MotorOutput.Inverted = DrivetrainConstants.leftInvertedState
        rightConfig.MotorOutput.Inverted = DrivetrainConstants.rightInvertedState

        leftConfig.CurrentLimits.StatorCurrentLimit = DrivetrainConstants.STATOR_CURRENT_LIMIT
        rightConfig.CurrentLimits.StatorCurrentLimit = DrivetrainConstants.STATOR_CURRENT_LIMIT

        leftConfig.CurrentLimits.SupplyCurrentLimit = DrivetrainConstants.SUPPLY_CURRENT_LIMIT
        rightConfig.CurrentLimits.SupplyCurrentLimit = DrivetrainConstants.SUPPLY_CURRENT_LIMIT

        leftLeader.configurator.apply(leftConfig)
        leftFollower.configurator.apply(leftConfig)
        rightFollower.configurator.apply(rightConfig)
        rightFollower.configurator.apply(rightConfig)
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
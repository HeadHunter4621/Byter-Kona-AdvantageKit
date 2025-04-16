package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units

class DrivetrainIOTalonFX: DrivetrainIO {

    private val leftLeader = TalonFX(DrivetrainConstants.LEFT_LEADER_ID)
    private val leftFollower = TalonFX(DrivetrainConstants.LEFT_FOLLOWER_ID)
    private val rightLeader = TalonFX(DrivetrainConstants.RIGHT_LEADER_ID)
    private val rightFollower = TalonFX(DrivetrainConstants.RIGHT_FOLLOWER_ID)

    private var leftConfig = TalonFXConfiguration()
    private var rightConfig = TalonFXConfiguration()

    init {

        val config = TalonFXConfiguration()
        config.CurrentLimits.SupplyCurrentLimit = DrivetrainConstants.SUPPLY_CURRENT_LIMIT
        config.CurrentLimits.SupplyCurrentLimitEnable = true
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake
        config.Feedback.SensorToMechanismRatio = DrivetrainConstants.OUTPUT_MULTIPLY
        config.Slot0.kP = DrivetrainConstants.P
        config.Slot0.kD = DrivetrainConstants.D

        config.MotorOutput.Inverted =
            if (DrivetrainConstants.LEFT_INVERTED)
                InvertedValue.Clockwise_Positive
            else
                InvertedValue.CounterClockwise_Positive
        leftLeader.configurator.apply(config, 0.25)
        leftFollower.configurator.apply(config, 0.25)

        config.MotorOutput.Inverted =
            if (DrivetrainConstants.RIGHT_INVERTED) {
                InvertedValue.Clockwise_Positive
            } else {
                InvertedValue.CounterClockwise_Positive
            }
        rightLeader.configurator.apply(config, 0.25)
        rightFollower.configurator.apply(config, 0.25)

        leftFollower.setControl(Follower(leftLeader.deviceID, false))
        rightFollower.setControl(Follower(rightLeader.deviceID, false))

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            leftLeader.position,
            leftLeader.velocity,
            leftLeader.motorVoltage,
            leftLeader.statorCurrent, // May need to be supplyCurrent
            leftFollower.statorCurrent, // May need to be supplyCurrent
            rightLeader.position,
            rightLeader.velocity,
            rightLeader.motorVoltage,
            rightLeader.statorCurrent, // May need to be supplyCurrent
            rightFollower.statorCurrent // May need to be supplyCurrent
        )
        leftLeader.optimizeBusUtilization()
        leftFollower.optimizeBusUtilization()
        rightLeader.optimizeBusUtilization()
        rightFollower.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: DrivetrainIO.DrivetrainIOInputs) {
        BaseStatusSignal.refreshAll(
            leftLeader.position,
            leftLeader.velocity,
            leftLeader.motorVoltage,
            leftLeader.supplyCurrent,
            leftFollower.supplyCurrent,
            rightLeader.position,
            rightLeader.position,
            rightLeader.motorVoltage,
            rightLeader.supplyCurrent,
            rightFollower.supplyCurrent
        )

        inputs.leftPositionRad = Units.rotationsToRadians(leftLeader.position.valueAsDouble)
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftLeader.velocity.valueAsDouble)
        inputs.leftAppliedVolts = leftLeader.motorVoltage.valueAsDouble
        inputs.leftCurrentAmps = doubleArrayOf(
            leftLeader.supplyCurrent.valueAsDouble,
            leftLeader.supplyCurrent.valueAsDouble
        )

        inputs.rightPositionRad = Units.rotationsToRadians(rightLeader.position.valueAsDouble)
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightLeader.velocity.valueAsDouble)
        inputs.rightAppliedVolts = rightLeader.motorVoltage.valueAsDouble
        inputs.rightCurrentAmps = doubleArrayOf(
            rightLeader.supplyCurrent.valueAsDouble,
            rightFollower.supplyCurrent.valueAsDouble
        )
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
package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.subsystems.drivetrain.DrivetrainIO

/** This drive implementation is for Talon FXs driving motors like the Falon 500 or Kraken X60.  */
class DrivetrainIOTalonFX2 : DrivetrainIO {
    private val leftLeader = TalonFX(DrivetrainConstants.LEFT_LEADER_ID)
    private val leftFollower = TalonFX(DrivetrainConstants.LEFT_FOLLOWER_ID)
    private val rightLeader = TalonFX(DrivetrainConstants.RIGHT_LEADER_ID)
    private val rightFollower = TalonFX(DrivetrainConstants.RIGHT_FOLLOWER_ID)

    private val leftPosition: StatusSignal<Angle> = leftLeader.position
    private val leftVelocity: StatusSignal<AngularVelocity> = leftLeader.velocity
    private val leftAppliedVolts: StatusSignal<Voltage> = leftLeader.motorVoltage
    private val leftLeaderCurrent: StatusSignal<Current> = leftLeader.supplyCurrent
    private val leftFollowerCurrent: StatusSignal<Current> = leftFollower.supplyCurrent

    private val rightPosition: StatusSignal<Angle> = rightLeader.position
    private val rightVelocity: StatusSignal<AngularVelocity> = rightLeader.velocity
    private val rightAppliedVolts: StatusSignal<Voltage> = rightLeader.motorVoltage
    private val rightLeaderCurrent: StatusSignal<Current> = rightLeader.supplyCurrent
    private val rightFollowerCurrent: StatusSignal<Current> = rightFollower.supplyCurrent

    private val voltageRequest = VoltageOut(0.0)
    private val velocityRequest = VelocityVoltage(0.0)

    init {
        val config = TalonFXConfiguration()
        config.CurrentLimits.SupplyCurrentLimit = DrivetrainConstants.SUPPLY_CURRENT_LIMIT
        config.CurrentLimits.SupplyCurrentLimitEnable = true
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake
        config.Feedback.SensorToMechanismRatio = motorReduction
        config.Slot0.kP = realKp
        config.Slot0.kD = realKd

        config.MotorOutput.Inverted =
            if (DrivetrainConstants.in) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        tryUntilOk(5) { leftLeader.configurator.apply(config, 0.25) }
        tryUntilOk(5) { leftFollower.configurator.apply(config, 0.25) }

        config.MotorOutput.Inverted =
            if (rightInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        tryUntilOk(5) { rightLeader.configurator.apply(config, 0.25) }
        tryUntilOk(5) { rightFollower.configurator.apply(config, 0.25) }

        leftFollower.setControl(Follower(leftLeader.deviceID, false))
        rightFollower.setControl(Follower(rightLeader.deviceID, false))

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            leftPosition,
            leftVelocity,
            leftAppliedVolts,
            leftLeaderCurrent,
            leftFollowerCurrent,
            rightPosition,
            rightVelocity,
            rightAppliedVolts,
            rightLeaderCurrent,
            rightFollowerCurrent
        )
        leftLeader.optimizeBusUtilization()
        leftFollower.optimizeBusUtilization()
        rightLeader.optimizeBusUtilization()
        rightFollower.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: DriveIOInputs) {
        BaseStatusSignal.refreshAll(
            leftPosition,
            leftVelocity,
            leftAppliedVolts,
            leftLeaderCurrent,
            leftFollowerCurrent,
            rightPosition,
            rightVelocity,
            rightAppliedVolts,
            rightLeaderCurrent,
            rightFollowerCurrent
        )

        inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.valueAsDouble)
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.valueAsDouble)
        inputs.leftAppliedVolts = leftAppliedVolts.valueAsDouble
        inputs.leftCurrentAmps =
            doubleArrayOf(leftLeaderCurrent.valueAsDouble, leftFollowerCurrent.valueAsDouble)

        inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.valueAsDouble)
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.valueAsDouble)
        inputs.rightAppliedVolts = rightAppliedVolts.valueAsDouble
        inputs.rightCurrentAmps =
            doubleArrayOf(
                rightLeaderCurrent.valueAsDouble, rightFollowerCurrent.valueAsDouble
            )
    }

    override fun setVoltage(leftVolts: Double, rightVolts: Double) {
        leftLeader.setControl(voltageRequest.withOutput(leftVolts))
        rightLeader.setControl(voltageRequest.withOutput(rightVolts))
    }

    override fun setVelocity(
        leftRadPerSec: Double, rightRadPerSec: Double, leftFFVolts: Double, rightFFVolts: Double
    ) {
        leftLeader.setControl(
            velocityRequest
                .withVelocity(Units.radiansToRotations(leftRadPerSec))
                .withFeedForward(leftFFVolts)
        )
        rightLeader.setControl(
            velocityRequest
                .withVelocity(Units.radiansToRotations(rightRadPerSec))
                .withFeedForward(rightFFVolts)
        )
    }
}
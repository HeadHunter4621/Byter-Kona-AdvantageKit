package frc.robot.subsystems.drivetrain

import org.team9432.annotation.Logged

interface DrivetrainIO {
    @Logged
    open class DrivetrainIOInputs {
        var leftPositionRad: Double = 0.0
        var leftVelocityRadPerSec: Double = 0.0
        var leftAppliedVolts: Double = 0.0
        var leftCurrentAmps: DoubleArray = doubleArrayOf()

        var rightPositionRad: Double = 0.0
        var rightVelocityRadPerSec: Double = 0.0
        var rightAppliedVolts: Double = 0.0
        var rightCurrentAmps: DoubleArray = doubleArrayOf()
    }

    fun updateInputs(inputs: DrivetrainIOInputs) {}

    fun setSpeed(leftSpeed: Double, rightSpeed: Double) {}

    fun setVoltage(leftVolts: Double, rightVolts: Double) {}

    fun stop() {}

}
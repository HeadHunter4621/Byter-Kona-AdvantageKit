package frc.robot.subsystems.drivetrain

import org.team9432.annotation.Logged

interface DrivetrainIO {
    @Logged
    open class DrivetrainIOInputs {
        var leftLeaderAppliedVolts = 0.0
        var leftFollowerAppliedVolts = 0.0

        var rightLeaderAppliedVolts = 0.0
        var rightFollowerAppliedVolts = 0.0
    }

    fun updateInputs(inputs: DrivetrainIOInputs) {}

    fun setSpeed(leftSpeed: Double, rightSpeed: Double) {}

    fun setVoltage(leftVolts: Double, rightVolts: Double) {}

    fun stop() {}

}
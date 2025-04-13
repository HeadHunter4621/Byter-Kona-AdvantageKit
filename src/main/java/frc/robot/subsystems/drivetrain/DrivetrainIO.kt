package frc.robot.subsystems.drivetrain

import org.littletonrobotics.junction.AutoLog

interface DrivetrainIO {
    @AutoLog
    class DrivetrainIOInputs {
        var leftLeaderAppliedVolts = 0.0
        var leftFollowerAppliedVolts = 0.0

        var rightLeaderAppliedVolts = 0.0
        var rightFollowerAppliedVolts = 0.0
    }

    fun updateInputs(inputs: DrivetrainIOInputs?) {}

    fun setVoltage(leftVolts: Double, rightVolts: Double) {}

}
package frc.robot.subsystems.drivetrain

import org.team9432.annotation.Logged

interface DrivetrainIO {
    @Logged
    open class DrivetrainIOInputs {

    }

    fun updateInputs(inputs: DrivetrainIOInputs) {}

    fun setSpeed(leftSpeed: Double, rightSpeed: Double) {}

    fun setVoltage(leftVolts: Double, rightVolts: Double) {}

    fun stop() {}

}
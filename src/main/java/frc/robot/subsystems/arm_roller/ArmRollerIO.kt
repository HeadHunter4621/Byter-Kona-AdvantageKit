package frc.robot.subsystems.arm_roller

import org.team9432.annotation.Logged

interface ArmRollerIO {

    @Logged
    open class ArmRollerIOInputs {
        var appliedVolts = 0.0
    }

    fun updateInputs(inputs: ArmRollerIOInputs) {}

    fun setSpeed(speed: Double) {}

    fun setVoltage(volts: Double) {}

    fun stop() {}

}
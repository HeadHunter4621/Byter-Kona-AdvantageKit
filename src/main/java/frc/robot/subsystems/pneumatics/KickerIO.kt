package frc.robot.subsystems.pneumatics

import org.team9432.annotation.Logged

interface KickerIO {

    @Logged
    open class KickerIOInputs {
        var kickerState = 0
    }

    fun updateInputs(inputs: KickerIOInputs) {}

    fun kickerOut() {}

    fun kickerIn() {}

    fun kickerOff() {}
}
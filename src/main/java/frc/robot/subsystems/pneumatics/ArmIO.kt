package frc.robot.subsystems.pneumatics

import org.team9432.annotation.Logged

interface ArmIO {

    @Logged
    open class ArmIOInputs {
        var armState = 0
    }

    fun updateInputs(inputs: ArmIOInputs) {}

    fun armUp() {}

    fun armDown() {}

    fun armOff() {}
}
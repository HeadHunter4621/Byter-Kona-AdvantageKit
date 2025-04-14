package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.arm_roller.ArmRoller
import frc.robot.subsystems.pneumatics.Arm

object Input {

    private val controller = CommandXboxController(0)

    /** Button Assignment **/
    // Arm
    private val rollerIntakeButton = controller.rightTrigger()    // Intake through Roller -> RT
    private val rollerOuttakeButton = controller.rightBumper()    // Outtake through Roller -> RB

    init {
        /** Command Assignment **/
        rollerIntakeButton.whileTrue(
            Arm.armDownCommand().andThen(
                ArmRoller.intakeCommand()))
            .onFalse(
                Arm.armUpCommand().andThen(
                    ArmRoller.stopRollerCommand()))

        rollerOuttakeButton.whileTrue(
            Arm.armDownCommand().andThen(
                ArmRoller.outtakeCommand()))
            .onFalse(
                Arm.armUpCommand().andThen(
                    ArmRoller.stopRollerCommand()))
    }

    // Joysticks
    fun getLeftX(): Double {
        return controller.leftX
    }

    fun getLeftY(): Double {
        return controller.leftY
    }

    fun getRightX(): Double {
        return controller.rightX
    }

    fun getRightY(): Double {
        return controller.rightY
    }
}
package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

object Input {

    private val controller = CommandXboxController(0)

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
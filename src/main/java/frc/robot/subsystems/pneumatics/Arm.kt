package frc.robot.subsystems.pneumatics

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

object Arm: SubsystemBase() {

    private val io = ArmIOCTREPCM()
    private val ioInputs = LoggedArmIOInputs()

    init {
        defaultCommand = armUpCommand()
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Arm", ioInputs)
    }

    fun armUp() {
        io.armUp()
    }

    fun armDown() {
        io.armDown()
    }

    fun armOff() {
        io.armOff()
    }

    /** Command Factories **/
    fun armUpCommand(): Command {
        return run{
            armUp()
        }
    }

    fun armDownCommand(): Command {
        return run{
            armDown()
        }
    }

    fun armOffCommand(): Command {
        return run{
            armOff()
        }
    }
}
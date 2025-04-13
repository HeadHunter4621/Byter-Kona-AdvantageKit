package frc.robot.subsystems.pneumatics

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

object Kicker: SubsystemBase() {

    private val io = KickerIOCTREPCM()
    private val ioInputs = LoggedKickerIOInputs()

    init {
        defaultCommand = kickerInCommand()
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Kicker", ioInputs)
    }

    fun kickerIn() {
        io.kickerIn()
    }

    fun kickerOut() {
        io.kickerOut()
    }

    fun kickerOff() {
        io.kickerOff()
    }

    /** Command Factories **/
    fun kickerInCommand(): Command {
        return run {
            kickerIn()
        }
    }

    fun kickerOutCommand(): Command {
        return run {
            kickerOut()
        }
    }

    fun kickerOffCommand(): Command {
        return run {
            kickerOff()
        }
    }
}
package frc.robot.subsystems.arm_roller

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.EditingConstants
import org.littletonrobotics.junction.Logger

object ArmRoller: SubsystemBase() {

    private val io = ArmRollerIOTalonSRX()
    private val ioInputs = LoggedArmRollerIOInputs()

    init {}

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Arm Roller", ioInputs)
    }

    fun setSpeed(speed: Double) {
        io.setSpeed(speed)
    }

    fun setVoltage(volts: Double) {
        io.setVoltage(volts)
    }

    fun stop() {
        io.stop()
    }

    /** Command Factories **/
    fun intakeCommand(): Command {
        return run {
            setSpeed(EditingConstants.INTAKE_SPEED)
        }
    }

    fun outtakeCommand(): Command {
        return run {
            setSpeed(EditingConstants.OUTTAKE_SPEED)
        }
    }

    fun stopRollerCommand(): Command {
        return run {
            stop()
        }
    }
}
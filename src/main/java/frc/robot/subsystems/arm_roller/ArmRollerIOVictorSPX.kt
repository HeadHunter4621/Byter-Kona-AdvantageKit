package frc.robot.subsystems.arm_roller

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import frc.robot.ArmRollerConstants
import frc.robot.subsystems.arm_roller.ArmRollerIO.ArmRollerIOInputs

class ArmRollerIOVictorSPX: ArmRollerIO {

    private val motor = WPI_VictorSPX(ArmRollerConstants.ARM_ROLLER_ID)

    init {
        motor.inverted = ArmRollerConstants.ARM_ROLLER_INVERTED
    }

    override fun updateInputs(inputs: ArmRollerIOInputs) {
        inputs.appliedVolts = motor.motorOutputVoltage
    }

    override fun setSpeed(speed: Double) {
        motor.set(speed)
    }

    override fun setVoltage(volts: Double) {
        motor.setVoltage(volts)
    }

    override fun stop() {
        motor.stopMotor()
    }
}
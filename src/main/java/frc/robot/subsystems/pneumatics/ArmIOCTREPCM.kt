package frc.robot.subsystems.pneumatics

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import frc.robot.PneumaticsConstants

class ArmIOCTREPCM: ArmIO {

    private val solenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.ARM_FORWARD_CHANNEL, PneumaticsConstants.ARM_REVERSE_CHANNEL)
    var armState = 0

    init {}

    override fun updateInputs(inputs: ArmIO.ArmIOInputs) {
        when(solenoid.get()){
            DoubleSolenoid.Value.kForward -> armState = 1
            DoubleSolenoid.Value.kOff -> armState = 0
            DoubleSolenoid.Value.kReverse -> armState = -1
            else -> armState = 0
        }
        inputs.armState = armState
    }

    override fun armUp() {
        solenoid.set(DoubleSolenoid.Value.kForward)
    }

    override fun armDown() {
        solenoid.set(DoubleSolenoid.Value.kReverse)
    }

    override fun armOff() {
        solenoid.set(DoubleSolenoid.Value.kOff)
    }
}
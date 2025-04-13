package frc.robot.subsystems.pneumatics

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import frc.robot.PneumaticsConstants

class KickerIOCTREPCM: KickerIO {

    private val solenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.KICKER_FORWARD_CHANNEL, PneumaticsConstants.KICKER_REVERSE_CHANNEL)
    var kickerState = 0

    init {}

    override fun updateInputs(inputs: KickerIO.KickerIOInputs) {
        when(solenoid.get()){
            DoubleSolenoid.Value.kForward -> kickerState = 1
            DoubleSolenoid.Value.kOff -> kickerState = 0
            DoubleSolenoid.Value.kReverse -> kickerState = -1
            else -> kickerState = 0
        }
        inputs.kickerState = kickerState
    }

    override fun kickerOut() {
        solenoid.set(DoubleSolenoid.Value.kForward)
    }

    override fun kickerIn() {
        solenoid.set(DoubleSolenoid.Value.kReverse)
    }

    override fun kickerOff() {
        solenoid.set(DoubleSolenoid.Value.kOff)
    }
}
package frc.robot.subsystems.pneumatics

import org.team9432.annotation.Logged

interface CompressorIO {

    @Logged
    open class CompressorIOInputs {
        var compressorOn = false
        var pressureSwitchValue = false
        var compressorCurrent = 0.0
    }

    fun updateInputs(inputs: CompressorIOInputs) {}

    fun enableCompressor() {}

    fun disableCompressor() {}
}
package frc.robot.subsystems.pneumatics

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.PneumaticsModuleType

class CompressorIOCTREPCM: CompressorIO {
    private val compressor = Compressor(PneumaticsModuleType.CTREPCM)
    private var compressorOn = false
    private var pneumaticsPressurized = false
    private var compressorCurrent = 0.0

    override fun updateInputs(inputs: CompressorIO.CompressorIOInputs) {
        inputs.compressorOn = compressorOn
        inputs.pressureSwitchValue = pneumaticsPressurized
        inputs.compressorCurrent = compressorCurrent
    }

    override fun enableCompressor() {
        compressor.enableDigital()
    }

    override fun disableCompressor() {
        compressor.disable()
    }
}
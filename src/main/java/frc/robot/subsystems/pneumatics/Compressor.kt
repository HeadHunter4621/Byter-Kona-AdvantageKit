package frc.robot.subsystems.pneumatics

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

object Compressor: SubsystemBase() {

    private val io = CompressorIOCTREPCM()
    private val ioInputs = LoggedCompressorIOInputs()

    init {}

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Compressor", ioInputs)
    }

    fun enableCompressor() {
        io.enableCompressor()
    }

    fun disableCompressor() {
        io.disableCompressor()
    }
}
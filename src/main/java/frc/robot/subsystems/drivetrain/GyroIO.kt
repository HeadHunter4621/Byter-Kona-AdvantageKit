package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface GyroIO {
    @Logged
    open class GyroIOInputs{
        var connected = false
        var yawPosition = Rotation2d()
        var yawVelocityRadPerSec = 0.0
    }

    fun updateInputs(inputs: GyroIOInputs){}
}
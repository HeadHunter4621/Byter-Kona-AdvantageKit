package frc.robot.subsystems.drivetrain

import com.studica.frc.AHRS
import com.studica.frc.AHRS.NavXComType
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units

class GyroIONavX {
    private val navX = AHRS(NavXComType.kMXP_SPI)

    @Override
    fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = navX.isConnected
        inputs.yawPosition = Rotation2d.fromDegrees(-navX.angle)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.rawGyroZ.toDouble())
    }
}
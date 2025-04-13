package org.sert2521.reefscape2025.subsystems.drivetrain

import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.drivetrain.DrivetrainConstants
import frc.robot.subsystems.drivetrain.GyroIO

class GyroIONavX: GyroIO {
    /* If you're here and have a problem with the gyro, just call software lead */
    private val imu = AHRS(AHRS.NavXComType.kUSB1, DrivetrainConstants.ODOMETRY_FREQUENCY)

    override fun updateInputs(inputs: GyroIO.GyroIOInputs){
        inputs.connected = imu.isConnected
        inputs.yawPosition = Rotation2d.fromDegrees(-imu.angle)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-imu.rate)
    }
}
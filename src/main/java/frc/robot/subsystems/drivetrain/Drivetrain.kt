package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.subsystems.drivetrain.GyroIONavX

object Drivetrain: SubsystemBase() {
    private var io: DrivetrainIO? = DrivetrainIOTalonFX()
    private val inputs: LoggedDrivetrainIOInputs = LoggedDrivetrainIOInputs()
    private var gyroIO: GyroIO? = GyroIONavX()
    private val gyroInputs: LoggedGyroIOInputs = LoggedGyroIOInputs()

    private val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH)
    private val poseEstimator = DifferentialDrivePoseEstimator(kinematics, Rotation2d(), 0.0, 0.0, Pose2d())
    private var rawGyroRotation = Rotation2d()

    override fun periodic() {
        io?.updateInputs(inputs)
        gyroIO?.updateInputs(gyroInputs)
        Logger.processInputs("Drive", inputs)
        Logger.processInputs("Drive/Gyro", inputs)
        rawGyroRotation = gyroInputs.yawPosition
    }

    /** Runs the drivetrain in open loop.  */
    fun drive(leftSpeed: Double, rightSpeed: Double) {
        io?.setSpeed(leftSpeed , rightSpeed)
    }

    /** Stops the drivetrain.  */
    fun stop() {
        io?.stop()
    }

    /** Returns the current odometry pose.  */
    @AutoLogOutput(key = "Odometry/Robot")
    fun getPose(): Pose2d {
        return poseEstimator.estimatedPosition
    }

    /** Returns the current odometry rotation.  */
    fun getRotation(): Rotation2d {
        return getPose().rotation
    }

}
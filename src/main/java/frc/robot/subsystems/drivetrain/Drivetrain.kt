package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.subsystems.drivetrain.DrivetrainConstants
import frc.robot.MetaConstants
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

object Drivetrain: SubsystemBase() {
    private var io: DrivetrainIO? = null
    private val inputs: LoggedDrivetrainIOInputs = LoggedDrivetrainIOInputs()
    private var gyroIO: GyroIO? = null
    private val gyroInputs: GyroIOInputsAutoLogged = GyroIOInputsAutoLogged()

    private val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH)
    private val poseEstimator = DifferentialDrivePoseEstimator(kinematics, Rotation2d(), 0.0, 0.0, Pose2d())
    private var sysId: SysIdRoutine? = null
    private var rawGyroRotation = Rotation2d()
    private var lastLeftPositionMeters = 0.0
    private var lastRightPositionMeters = 0.0

    fun Drive(io: DrivetrainIO?, gyroIO: GyroIO?) {
        this.io = io
        this.gyroIO = gyroIO

        // Configure SysId
        sysId =
            SysIdRoutine(
                SysIdRoutine.Config(
                    null,
                    null,
                    null
                ) { state: SysIdRoutineLog.State ->
                    Logger.recordOutput(
                        "Drive/SysIdState",
                        state.toString()
                    )
                },
                Mechanism(
                    { voltage: Voltage -> runOpenLoop(voltage.`in`(Units.Volts), voltage.`in`(Units.Volts)) }, null,
                    this
                )
            )
    }

    override fun periodic() {
        io?.updateInputs(inputs)
        gyroIO?.updateInputs(gyroInputs)
        Logger.processInputs("Drive", inputs)
        Logger.processInputs("Drive/Gyro", inputs)

        // Update gyro angle
        if (gyroInputs.connected) {
            // Use the real gyro angle
            rawGyroRotation = gyroInputs.yawPosition
        } else {
            // Use the angle delta from the kinematics and module deltas
            val twist =
                kinematics.toTwist2d(
                    getLeftPositionMeters() - lastLeftPositionMeters,
                    getRightPositionMeters() - lastRightPositionMeters
                )
            rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
            lastLeftPositionMeters = getLeftPositionMeters()
            lastRightPositionMeters = getRightPositionMeters()
        }

        // Update odometry
        poseEstimator.update(rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters())
    }

    /** Runs the drive in open loop.  */
    fun runOpenLoop(leftVolts: Double, rightVolts: Double) {
        io?.setVoltage(leftVolts, rightVolts)
    }

    /** Stops the drive.  */
    fun stop() {
        runOpenLoop(0.0, 0.0)
    }

    /** Returns a command to run a quasistatic test in the specified direction.  */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return sysId!!.quasistatic(direction)
    }

    /** Returns a command to run a dynamic test in the specified direction.  */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return sysId!!.dynamic(direction)
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

    /** Resets the current odometry pose.  */
    fun setPose(pose: Pose2d?) {
        poseEstimator.resetPosition(
            rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters(), pose
        )
    }

    /** Returns the position of the left wheels in meters.  */
    @AutoLogOutput
    fun getLeftPositionMeters(): Double {
        return inputs.leftPositionRad * DrivetrainConstants.WHEEL_RADIUS
    }

    /** Returns the position of the right wheels in meters.  */
    @AutoLogOutput
    fun getRightPositionMeters(): Double {
        return inputs.rightPositionRad * DrivetrainConstants.WHEEL_RADIUS
    }

    /** Returns the velocity of the left wheels in meters/second.  */
    @AutoLogOutput
    fun getLeftVelocityMetersPerSec(): Double {
        return inputs.leftVelocityRadPerSec * DrivetrainConstants.WHEEL_RADIUS
    }

    /** Returns the velocity of the right wheels in meters/second.  */
    @AutoLogOutput
    fun getRightVelocityMetersPerSec(): Double {
        return inputs.rightVelocityRadPerSec * DrivetrainConstants.WHEEL_RADIUS
    }

    /** Returns the average velocity in radians/second.  */
    fun getCharacterizationVelocity(): Double {
        return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0
    }
}
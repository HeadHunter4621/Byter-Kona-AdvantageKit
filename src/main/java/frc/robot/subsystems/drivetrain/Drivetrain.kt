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
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

object Drivetrain: SubsystemBase() {
    private var io: DrivetrainIO? = null
    private val inputs: LoggedDrivetrainIOInputs = LoggedDrivetrainIOInputs()
    private var gyroIO: GyroIO? = null
    private val gyroInputs: LoggedGyroIOInputs = LoggedGyroIOInputs()

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
        rawGyroRotation = gyroInputs.yawPosition
    }

    /** Runs the drive in open loop.  */
    fun drive(leftSpeed: Double, rightSpeed: Double) {
        io?.setSpeed(leftSpeed , rightSpeed)
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

}
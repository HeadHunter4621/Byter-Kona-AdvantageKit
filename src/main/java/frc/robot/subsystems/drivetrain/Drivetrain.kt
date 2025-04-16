package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.subsystems.drivetrain.GyroIONavX


object Drivetrain: SubsystemBase() {
    private var io: DrivetrainIO? = DrivetrainIOTalonFX()
    private val inputs: LoggedDrivetrainIOInputs = LoggedDrivetrainIOInputs()
    private var gyroIO: GyroIO? = GyroIONavX()
    private val gyroInputs: LoggedGyroIOInputs = LoggedGyroIOInputs()
    private var sysId: SysIdRoutine? = null

    private val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH)
    private val poseEstimator = DifferentialDrivePoseEstimator(kinematics, Rotation2d(), 0.0, 0.0, Pose2d())
    private var rawGyroRotation = Rotation2d()

    fun Drive(io: DrivetrainIO?, gyroIO: GyroIO?) {
        this.io = io
        this.gyroIO = gyroIO

        // Configure SysId
        sysId = SysIdRoutine(
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
                    { voltage: Voltage -> driveVolts(voltage.`in`(Volts), voltage.`in`(Volts)) }, null,
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

    /** Runs the drivetrain in open loop.  */
    fun drive(leftSpeed: Double, rightSpeed: Double) {
        io?.setSpeed(leftSpeed , rightSpeed)
    }

    fun driveVolts(leftVoltage: Double, rightVoltage: Double) {
        io?.setVoltage(leftVoltage, rightVoltage)
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
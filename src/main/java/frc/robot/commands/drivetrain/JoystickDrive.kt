
package frc.robot.commands.drivetrain

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Input
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.drivetrain.DrivetrainConstants
import java.util.*

object JoystickDriveDrive: Command() {

    private val driveSlewRateLimiter = SlewRateLimiter(DrivetrainConstants.SLEW_RATE_LIMIT)
    private val turnSlewRatelimiter = SlewRateLimiter(DrivetrainConstants.SLEW_RATE_LIMIT)

    // private val feedForward = DifferentialDriveFeedforward(DrivetrainConstants.V_LINEAR, DrivetrainConstants.A_LINEAR, DrivetrainConstants.V_ANGULAR, DrivetrainConstants.A_ANGULAR, DrivetrainConstants.TRACK_WIDTH)
    // private val pid = PIDController(DrivetrainConstants.P, DrivetrainConstants.I, DrivetrainConstants.D)

    private var leftSpeedOut = 0.0
    private var rightSpeedOut = 0.0

    init {
        addRequirements(Drivetrain)
    }

    private fun deadband(value: Double, range: Double): Double {
        if (value < range && value > -range) {
            return 0.0
        } else {
            return value
        }
    }

    override fun initialize() {

        driveSlewRateLimiter.reset(0.0)
        turnSlewRatelimiter.reset(0.0)
    }

    override fun execute() {
        val yIn = driveSlewRateLimiter.calculate(DrivetrainConstants.OUTPUT_MULTIPLY * (Input.getRightY()))
        val xIn = turnSlewRatelimiter.calculate(DrivetrainConstants.OUTPUT_MULTIPLY * (Input.getLeftX()))

        leftSpeedOut = yIn * DrivetrainConstants.MAX_DRIVE_SPEED + xIn * DrivetrainConstants.MAX_TURN_SPEED
        rightSpeedOut = yIn * DrivetrainConstants.MAX_DRIVE_SPEED - xIn * DrivetrainConstants.MAX_TURN_SPEED

        Drivetrain.drive(leftSpeedOut, rightSpeedOut)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
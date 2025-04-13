package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import kotlin.random.Random

object EditingConstants {
    const val INTAKE_SPEED = 1.0
    const val OUTTAKE_SPEED = -1.0
    const val MAX_DRIVE_SPEED = 2
    const val MAX_TURN_SPEED = 2
}

object PneumaticsConstants {
    const val ARM_FORWARD_CHANNEL = -1
    const val ARM_REVERSE_CHANNEL = -1
    const val KICKER_FORWARD_CHANNEL = -1
    const val KICKER_REVERSE_CHANNEL = -1
}

object ArmRollerConstants {
    const val ARM_ROLLER_ID = -1
    const val ARM_ROLLER_INVERTED = false
}

object MetaConstants {
    enum class Mode{
        REAL,
        SIM,
        REPLAY
    }

    val atEvent = false
    private val simMode = Mode.REPLAY
    val currentMode = if (RobotBase.isReal()) Mode.REAL else simMode

    enum class Gender{
        MASCULINE,
        FEMININE,
        NONBINARY,
        DOESNT_CARE
    }

    val gender = Gender.entries[Random.nextInt().mod(4)]
}
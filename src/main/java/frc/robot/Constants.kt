package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import kotlin.random.Random

object EditingConstants {
    const val INTAKE_SPEED = 1.0
    const val OUTTAKE_SPEED = -1.0
}

object PneumaticsConstants {
    const val ARM_IN_ID = -1
    const val ARM_OUT_ID = -1
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
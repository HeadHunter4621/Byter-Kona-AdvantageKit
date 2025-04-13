package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import kotlin.random.Random

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
package org.firstinspires.ftc.teamcode.library.functions

enum class AllianceColor {
    RED, BLUE;

    companion object {
        @JvmField var persistingAllianceColor: AllianceColor = BLUE
    }
}

enum class StartingPosition {
    LEFT, RIGHT
}

enum class SignalState(var shift: Double) {
    LEFT(-20.0),
    CENTER(0.0),
    RIGHT(20.0)
}
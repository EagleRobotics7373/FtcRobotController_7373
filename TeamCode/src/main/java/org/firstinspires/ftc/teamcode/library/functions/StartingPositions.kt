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

enum class CameraPosition {
    LEFT, CENTER, RIGHT
}
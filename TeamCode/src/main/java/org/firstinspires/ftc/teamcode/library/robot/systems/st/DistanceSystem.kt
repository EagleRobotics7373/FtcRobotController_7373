package org.firstinspires.ftc.teamcode.library.robot.systems.st

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class DistanceSystem(
        private val leftDistanceSensor: DistanceSensor,
        private val rightDistanceSensor: DistanceSensor,
        private val frontDistanceSensor: DistanceSensor
) {
    fun getLeftDistance(): Double {
        return leftDistanceSensor.getDistance(DistanceUnit.INCH)
    }

    fun getRightDistance(): Double {
        return rightDistanceSensor.getDistance(DistanceUnit.INCH)
    }

    fun getFrontDistance(): Double {
        return frontDistanceSensor.getDistance(DistanceUnit.INCH)
    }

    fun getCoords(facing: Facing): Vector2d {
        return when (facing) {
            Facing.XPLUS -> Vector2d(72 - getFrontDistance(), 72 - getRightDistance())
            Facing.XMINUS -> Vector2d(getFrontDistance() - 72, 72 - getLeftDistance())
            Facing.YPLUS -> Vector2d(72 - getRightDistance(), 72 - getFrontDistance())
            Facing.YMINUS -> Vector2d(72 - getLeftDistance(), getFrontDistance() - 72)
        }
    }

    enum class Facing {
        XPLUS,
        XMINUS,
        YPLUS,
        YMINUS
    }
}
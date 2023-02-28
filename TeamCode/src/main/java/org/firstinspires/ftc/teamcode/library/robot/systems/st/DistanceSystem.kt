package org.firstinspires.ftc.teamcode.library.robot.systems.st

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs

class DistanceSystem(
        private val leftDistanceSensor: DistanceSensor,
        private val rightDistanceSensor: DistanceSensor,
        private val frontDistanceSensor: DistanceSensor
) {

    fun getFrontDistance(expectedValue: Double = Double.NaN, errorThreshold: Double = 1.5): Double {
        val distance = frontDistanceSensor.getDistance(DistanceUnit.INCH)
        if (!expectedValue.isNaN()) {
            if (abs(distance - expectedValue) > errorThreshold) {
                return expectedValue
            }
        }
        return distance
    }

    fun getLeftDistance(expectedValue: Double = Double.NaN, errorThreshold: Double = 1.5): Double {
        val distance = leftDistanceSensor.getDistance(DistanceUnit.INCH)
        if (!expectedValue.isNaN()) {
            if (abs(distance - expectedValue) > errorThreshold) {
                return expectedValue
            }
        }
        return distance
    }

    fun getRightDistance(expectedValue: Double = Double.NaN, errorThreshold: Double = 2.0): Double {
        val distance = rightDistanceSensor.getDistance(DistanceUnit.INCH)
        if (!expectedValue.isNaN()) {
            if (abs(distance - expectedValue) > errorThreshold) {
                return expectedValue
            }
        }
        return distance
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
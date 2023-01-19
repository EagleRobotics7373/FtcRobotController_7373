package org.firstinspires.ftc.teamcode.library.robot.systems.lt

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.robot.systems.meet1.LiftClawSystem

class DualServoClawLift(
        private val linearActuatorMotor: DcMotor,
        private val leftClawServo: Servo,
        private val rightClawServo: Servo
) {
    var liftPosition: LiftPosition = LiftPosition.FLOOR

    fun liftManual(motorPower: Double) {
        linearActuatorMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        linearActuatorMotor.power = motorPower
    }

    fun liftAuto(ticks: Int, power: Double) {
        linearActuatorMotor.targetPosition = ticks
        linearActuatorMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        linearActuatorMotor.power = power
    }

    fun liftAuto(newLiftPosition: LiftPosition, power: Double) {
        linearActuatorMotor.targetPosition = newLiftPosition.ticks
        linearActuatorMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        linearActuatorMotor.power = power
        liftPosition = newLiftPosition
    }

    fun liftCycleUp(power: Double) {
        when (liftPosition) {
            LiftPosition.FLOOR -> liftAuto(LiftPosition.GROUND, power)
            LiftPosition.GROUND -> liftAuto(LiftPosition.LOW, power)
            LiftPosition.LOW -> liftAuto(LiftPosition.MIDDLE, power)
            LiftPosition.MIDDLE -> liftAuto(LiftPosition.HIGH, power)
            LiftPosition.HIGH -> liftAuto(LiftPosition.HIGH, power)
        }
    }

    fun liftCycleDown(power: Double) {
        when (liftPosition) {
            LiftPosition.FLOOR -> liftAuto(LiftPosition.FLOOR, power)
            LiftPosition.GROUND -> liftAuto(LiftPosition.FLOOR, power)
            LiftPosition.LOW -> liftAuto(LiftPosition.GROUND, power)
            LiftPosition.MIDDLE -> liftAuto(LiftPosition.LOW, power)
            LiftPosition.HIGH -> liftAuto(LiftPosition.HIGH, power)
        }
    }

    fun clawOpen() {
        leftClawServo.position = 0.8
        rightClawServo.position = 0.8
    }

    fun clawClose() {
        leftClawServo.position = 0.14
        rightClawServo.position = 0.1
    }

    enum class LiftPosition(val ticks: Int) {
        FLOOR(0),
        GROUND(-200),
        LOW(-1776),
        MIDDLE(-2925),
        HIGH(-4050);
    }
}
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

    private var openPosition: Array<Double> = arrayOf(0.93, 0.07)
    private var closedPosition: Array<Double> = arrayOf(0.45, 0.55)

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
        liftAuto(liftPosition.ticks, power)
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
        leftClawServo.position = openPosition[0]
        rightClawServo.position = openPosition[1]
    }

    fun clawClose() {
        leftClawServo.position = closedPosition[0]
        rightClawServo.position = closedPosition[1]
    }

    enum class LiftPosition(val ticks: Int) {
        FLOOR(0),
        GROUND(-100),
        LOW(-690),
        MIDDLE(-1000),
        HIGH(-1000);
    }
}
package org.firstinspires.ftc.teamcode.library.robot.systems.meet1

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo

class LiftClawSystem(
        private val linearActuatorMotor: DcMotor,
        private val liftMotor: DcMotor,
        private val clawServo: Servo
) {
    var liftPosition: LiftPosition = LiftPosition.GROUND

    private var openPosition: Double = 0.0
    private var closedPosition: Double = 1.0

    fun liftManual(motor1Power: Double, motor2Power: Double) {
        linearActuatorMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        linearActuatorMotor.power = motor1Power
        liftMotor.power = motor2Power
    }

    fun liftAuto(height: LiftPosition, power1: Double, power2: Double) {
        liftPosition = height
        linearActuatorMotor.targetPosition = height.ticks1
        liftMotor.targetPosition = height.ticks2

        linearActuatorMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        linearActuatorMotor.power = power1
        liftMotor.power = power2
    }

    fun liftCycleUp(power1: Double, power2: Double) {
        when (liftPosition) {
            LiftPosition.FLOOR -> liftAuto(LiftPosition.GROUND, power1, power2)
            LiftPosition.GROUND -> liftAuto(LiftPosition.LOW, power1, power2)
            LiftPosition.LOW -> liftAuto(LiftPosition.MIDDLE, power1, power2)
            LiftPosition.MIDDLE -> liftAuto(LiftPosition.HIGH, power1, power2)
            LiftPosition.HIGH -> liftAuto(LiftPosition.HIGH, power1, power2)
        }
    }

    fun liftCycleDown(power1: Double, power2: Double) {
        when (liftPosition) {
            LiftPosition.FLOOR -> liftAuto(LiftPosition.FLOOR, power1, power2)
            LiftPosition.GROUND -> liftAuto(LiftPosition.FLOOR, power1, power2)
            LiftPosition.LOW -> liftAuto(LiftPosition.GROUND, power1, power2)
            LiftPosition.MIDDLE -> liftAuto(LiftPosition.LOW, power1, power2)
            LiftPosition.HIGH -> liftAuto(LiftPosition.MIDDLE, power1, power2)
        }
    }

    fun clawOpen() {
        clawServo.position = openPosition
    }

    fun clawClose() {
        clawServo.position = closedPosition
    }

    enum class LiftPosition(val ticks1: Int, val ticks2: Int) {
        FLOOR(0, 0),
        GROUND(1, 0),
        LOW(2, 0),
        MIDDLE(3, 0),
        HIGH(4, 0);
    }
}
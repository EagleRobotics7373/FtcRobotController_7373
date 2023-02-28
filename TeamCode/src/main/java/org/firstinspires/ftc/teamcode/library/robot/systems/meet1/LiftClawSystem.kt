package org.firstinspires.ftc.teamcode.library.robot.systems.meet1

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo

class LiftClawSystem(
        private val linearActuatorMotor: DcMotor,
        private val liftMotor: DcMotor,
        private val clawServo: Servo
) {
    var liftPosition: LiftPosition = LiftPosition.GROUND

    private var openPosition: Double = 0.93
    private var closedPosition: Double = 0.45

    fun liftManual(motor1Power: Double, motor2Power: Double) {
        linearActuatorMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        linearActuatorMotor.power = motor1Power
        liftMotor.power = motor2Power
    }

    fun liftAuto(ticks1: Int, ticks2: Int, power1: Double, power2: Double) {
        linearActuatorMotor.targetPosition = ticks1
        liftMotor.targetPosition = ticks2

        linearActuatorMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        linearActuatorMotor.power = power1
        liftMotor.power = power2
    }

    fun liftCycleUp(power1: Double, power2: Double) {
        when (liftPosition) {
            LiftPosition.FLOOR -> liftAuto(LiftPosition.GROUND.ticks1, LiftPosition.GROUND.ticks2, power1, power2)
            LiftPosition.GROUND -> liftAuto(LiftPosition.LOW.ticks1, LiftPosition.LOW.ticks2, power1, power2)
            LiftPosition.LOW -> liftAuto(LiftPosition.MIDDLE.ticks1, LiftPosition.MIDDLE.ticks2, power1, power2)
            LiftPosition.MIDDLE -> liftAuto(LiftPosition.HIGH.ticks1, LiftPosition.HIGH.ticks2, power1, power2)
            LiftPosition.HIGH -> liftAuto(LiftPosition.HIGH.ticks1, LiftPosition.HIGH.ticks2, power1, power2)
        }
    }

    fun liftCycleDown(power1: Double, power2: Double) {
        when (liftPosition) {
            LiftPosition.FLOOR -> liftAuto(LiftPosition.FLOOR.ticks1, LiftPosition.FLOOR.ticks2, power1, power2)
            LiftPosition.GROUND -> liftAuto(LiftPosition.FLOOR.ticks1, LiftPosition.FLOOR.ticks2, power1, power2)
            LiftPosition.LOW -> liftAuto(LiftPosition.GROUND.ticks1, LiftPosition.GROUND.ticks2, power1, power2)
            LiftPosition.MIDDLE -> liftAuto(LiftPosition.LOW.ticks1, LiftPosition.LOW.ticks2, power1, power2)
            LiftPosition.HIGH -> liftAuto(LiftPosition.HIGH.ticks1, LiftPosition.HIGH.ticks2, power1, power2)
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
        GROUND(-100, 0),
        LOW(-690, 0),
        MIDDLE(-1000, 0),
        HIGH(-1000, 0);
    }
}
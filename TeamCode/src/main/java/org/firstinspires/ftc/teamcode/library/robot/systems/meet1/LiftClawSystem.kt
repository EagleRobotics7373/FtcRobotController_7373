package org.firstinspires.ftc.teamcode.library.robot.systems.meet1

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo

class LiftClawSystem(
        private val liftMotor1: DcMotor,
        private val liftMotor2: DcMotor,
        private val clawServo: Servo
) {

    fun LiftManual(motor1Power: Double, motor2Power: Double) {
        liftMotor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        liftMotor1.power = motor1Power
        liftMotor2.power = motor2Power
    }

    fun LiftAuto(height: LiftPosition, power1: Double, power2: Double) {
        liftMotor1.targetPosition = height.ticks1
        liftMotor2.targetPosition = height.ticks2

        liftMotor1.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor2.mode = DcMotor.RunMode.RUN_TO_POSITION

        liftMotor1.power = power1
        liftMotor2.power = power2
    }

    enum class LiftPosition(val ticks1: Int, val ticks2: Int) {
        GROUND(0, 0),
        LOW(1, 0),
        MIDDLE(2, 0),
        HIGH(3, 0);
    }

    enum class ClawPosition(val ticks: Int) {
        OPEN(0),
        CLOSED(1);
    }
}
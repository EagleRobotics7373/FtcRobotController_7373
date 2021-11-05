package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import kotlin.math.absoluteValue

@TeleOp(name="TeleOpCore (Kotlin)")
class TeleOpCoreKt: OpMode() {

    lateinit var robot: ExtThinBot
    val gamepad1Ex = GamepadEx(gamepad1)
    val gamepad2Ex = GamepadEx(gamepad2)

    override fun init() {
        robot = ExtThinBot(hardwareMap)
    }

    override fun loop() {

        gamepad1Ex.readButtons()
        gamepad2Ex.readButtons()

        // Control deposit servo
        when {
            gamepad2.a -> robot.outServo.position = 0.0
            gamepad2.b -> robot.outServo.position = 0.0
        }

        // Control first-stage intake motor
        robot.intakeMotor.power = when {
            gamepad2.left_trigger > 0.05 -> -gamepad2.left_trigger.toDouble()
            gamepad2.right_trigger > 0.05 -> gamepad2.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> gamepad1.left_trigger.toDouble()
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            else -> 0.0
        }

        // Control carousel motor
        robot.carouselMotor.power = when {
            gamepad2.left_stick_y.absoluteValue > 0.05 -> gamepad2.left_stick_y.toDouble()
            gamepad2.left_bumper -> defaultCarouselSpeed
            else -> 0.0
        }

        // Adjust default carousel speed
        when {
            gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> defaultCarouselSpeed += 0.05
            gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> defaultCarouselSpeed -= 0.05
            gamepad2Ex.wasJustPressed(GamepadKeys.Button.X) -> defaultCarouselSpeed *= -1.0
        }

        // Control linear actuator motor (not implemented in hardware)
        robot.linearActuatorMotor.power = gamepad2.right_stick_y * 0.20

        // Control linear actuator servo (not implemented in hardware)
        when {
            gamepad2.x -> robot.linearActuatorServo.position = LIN_POS_OUT
            gamepad2.y -> robot.linearActuatorServo.position = LIN_POS_IN
        }

        // Adjust drivetrain speed
        when {
            gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP) ->
                if (speed < 3) speed++
            gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) ->
                if (speed > 1) speed--
        }

        val vertical = -gamepad1.left_stick_y * speed * if (reverse) 1 else -1
        val horizontal = gamepad1.left_stick_x * speed * if (reverse) 1 else -1
        val pivot = gamepad1.right_stick_x * speed


//        robot.holonomic.runWithoutEncoderVectored(horizontal, vertical, pivot, 0);
        robot.frontRightMotor.power = pivot - vertical + horizontal
        robot.backRightMotor.power = pivot - vertical - horizontal
        robot.frontLeftMotor.power = pivot + vertical + horizontal
        robot.backLeftMotor.power = pivot + vertical - horizontal
    }
}
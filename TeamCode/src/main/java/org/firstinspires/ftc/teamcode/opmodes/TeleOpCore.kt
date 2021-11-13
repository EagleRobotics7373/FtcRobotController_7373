package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.library.functions.DashboardVar
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem
import kotlin.math.absoluteValue
import kotlin.math.max

@TeleOp(name="TeleOpCore (Kotlin)")
class TeleOpCore: OpMode() {

    lateinit var robot: ExtThinBot
    lateinit var gamepad1Ex: GamepadEx
    lateinit var gamepad2Ex: GamepadEx

    private var reverse by DashboardVar(false, "reverse", this::class)
    private var speed by DashboardVar(1, "speed", this::class) {it in 1..3}

    private var defaultCarouselSpeed by DashboardVar(-0.25, "defaultCarouselSpeed", this::class)
    private var maxCarouselSpeed by DashboardVar(0.6, "defaultCarouselSpeed", this::class) {it in 0.0..1.0}

    private var depositLiftPowerAuto by DashboardVar(0.5, "depositLiftPowerAuto", this::class) { it.absoluteValue <= 1.0}

    private var defaultWebcamPosition by DashboardVar(1.0, "defaultWebcamPosition", this::class) { it in 0.0..1.0 }

    override fun init() {
        robot = ExtThinBot(hardwareMap)
        gamepad1Ex = GamepadEx(gamepad1)
        gamepad2Ex = GamepadEx(gamepad2)
    }

    override fun loop() {

        gamepad1Ex.readButtons()
        gamepad2Ex.readButtons()

        // Control deposit servo
        when {
            gamepad2.a -> robot.fullIntakeSystem.depositServoIsExtended = true
            gamepad2.b -> robot.fullIntakeSystem.depositServoIsExtended = false
        }

        // Control both intake motors
        val dualIntakeMotorPower = when {
            gamepad2.right_trigger > 0.05 -> -gamepad2.right_trigger.toDouble()
            gamepad2.left_trigger > 0.05 -> gamepad2.left_trigger.toDouble()
            gamepad1.right_trigger > 0.05 -> -gamepad1.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> gamepad1.left_trigger.toDouble()
            else -> 0.0
        }
        if (dualIntakeMotorPower.absoluteValue > 0)
            robot.fullIntakeSystem.intakeManual(dualIntakeMotorPower)
        else if (!gamepad2.right_bumper)
            robot.fullIntakeSystem.intakeManual(gamepad2.left_stick_y.toDouble(), gamepad2.right_stick_y.toDouble())
        else
            robot.fullIntakeSystem.intakeManual(0.0)

        // Control carousel motor
        robot.carouselMotor.power = when {
            gamepad2.right_bumper && gamepad2.left_stick_y.absoluteValue > 0.05 ->
                gamepad2.left_stick_y.toDouble().coerceIn(-maxCarouselSpeed, maxCarouselSpeed)
            gamepad2.left_bumper ->
                defaultCarouselSpeed
            else -> 0.0
        }

        // Adjust default carousel speed
        if (gamepad2.right_bumper) {
            when {
                gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> defaultCarouselSpeed += 0.05
                gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> defaultCarouselSpeed -= 0.05
                gamepad2Ex.wasJustPressed(GamepadKeys.Button.X) -> defaultCarouselSpeed *= -1.0
            }
        }

        // Control deposit lift
        val depositLiftPower = if (gamepad2.right_bumper) gamepad2.right_stick_y.toDouble()*0.5 else 0.0
        if (robot.depositLiftMotor.mode == DcMotor.RunMode.RUN_TO_POSITION) {
            if (gamepad2.right_stick_y.absoluteValue > 0) robot.fullIntakeSystem.depositLiftManual(0.0)
        } else {
            robot.fullIntakeSystem.depositLiftManual(depositLiftPower)
        }

        if (!gamepad2.right_bumper) {
            when {
                gamepad2.dpad_down -> robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.LOW, depositLiftPowerAuto)
                gamepad2.dpad_right -> robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.MIDDLE, depositLiftPowerAuto)
                gamepad2.dpad_up -> robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.HIGH, depositLiftPowerAuto)
            }
        }
        if (gamepad2.y) robot.fullIntakeSystem.resetDepositZero()

        // Control webcam servo
        robot.webcamServo.position = defaultWebcamPosition

        // Adjust drivetrain speed
        when {
            gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> if (speed < 3) speed++
            gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> if (speed > 1) speed--
        }

        // Reverse drivetrain
        when {
            gamepad1.a -> reverse = false
            gamepad1.b -> reverse = true
            gamepad1Ex.wasJustPressed(GamepadKeys.Button.X) -> reverse = !reverse
        }

        val vertical = -gamepad1.left_stick_y.toDouble() * speed/3 * if (reverse) 1 else -1
        val horizontal = gamepad1.left_stick_x.toDouble() * speed/3 * if (reverse) 1 else -1
        val pivot = gamepad1.right_stick_x.toDouble() * speed/3


//        robot.holonomic.runWithoutEncoderVectored(horizontal, vertical, pivot, 0);
        robot.frontRightMotor.power = pivot - vertical + horizontal
        robot.backRightMotor.power = pivot - vertical - horizontal
        robot.frontLeftMotor.power = pivot + vertical + horizontal
        robot.backLeftMotor.power = pivot + vertical - horizontal

        telemetry.addData("Carousel speed", defaultCarouselSpeed)
        telemetry.addData("Carousel side", if (defaultCarouselSpeed < 0) "BLUE" else "RED")
        telemetry.addLine()
        telemetry.addData("Deposit lift power", depositLiftPower)
        telemetry.addData("Deposit lift position", robot.depositLiftMotor.currentPosition)
        telemetry.addData("Deposit lift mode", robot.depositLiftMotor.mode)
        telemetry.addData("Combined intake motor power", dualIntakeMotorPower)
        telemetry.addLine()
        telemetry.addData("gamepad2.right_trigger", gamepad2.right_trigger)
        telemetry.addData("gamepad2.left_trigger", gamepad2.left_trigger)
        telemetry.addData("gamepad1.right_trigger", gamepad1.right_trigger)
        telemetry.addData("gamepad1.left_trigger", gamepad1.left_trigger)

        robot.fullIntakeSystem.update()
    }
}
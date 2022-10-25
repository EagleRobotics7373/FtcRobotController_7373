package org.firstinspires.ftc.teamcode.testopmodes.seasontests

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot

@TeleOp(name="Lift Test", group="Test")
class LiftTest: OpMode() {

    lateinit var robot: ExtThinBot
    lateinit var gamepad1Ex: GamepadEx
    lateinit var gamepad2Ex: GamepadEx

    private lateinit var toggleGamepad1TouchpadPress: ToggleButtonWatcher
    private lateinit var toggleGamepad2TouchpadPress: ToggleButtonWatcher

    private var autoMode: Boolean = false

    private var linearActuatorPosition: Int = 0
    private var liftPosition: Int = 0

    override fun init() {
        robot = ExtThinBot(hardwareMap)

        gamepad1Ex = GamepadEx(gamepad1)
        gamepad2Ex = GamepadEx(gamepad2)

        toggleGamepad1TouchpadPress = ToggleButtonWatcher { gamepad1.touchpad }
        toggleGamepad2TouchpadPress = ToggleButtonWatcher { gamepad2.touchpad }

        robot.linearActuatorMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    override fun loop() {
        gamepad1Ex.readButtons()
        gamepad2Ex.readButtons()
        toggleGamepad1TouchpadPress()
        toggleGamepad2TouchpadPress()

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.A)) {
            autoMode = !autoMode
        }

        if (!autoMode) {
            robot.liftClawSystem.liftManual(-gamepad2.left_stick_y.toDouble(), -gamepad2.right_stick_y.toDouble())
        } else {
            robot.liftClawSystem.liftAuto(linearActuatorPosition, liftPosition, 1.0, 0.0)
        }

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            linearActuatorPosition += 25
        }
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            linearActuatorPosition -= 25
        }
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            linearActuatorPosition -= 5
        }
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            linearActuatorPosition += 5
        }

        robot.clawServo.position = 0.5

        telemetry.addData("Left Stick Y", gamepad2.left_stick_y)
        telemetry.addData("Right Stick Y", gamepad2.right_stick_y)
        telemetry.addLine()
        telemetry.addData("Linear Actuator Power", robot.linearActuatorMotor.power)
        telemetry.addData("Lift Power", robot.liftMotor.power)
        telemetry.addLine()
        telemetry.addData("Linear Actuator Position", robot.linearActuatorMotor.currentPosition)
        telemetry.addData("Linear Actuator Target Position", linearActuatorPosition)
        telemetry.addLine()
        telemetry.addData("Lift Position", robot.liftMotor.currentPosition)
        telemetry.addData("Lift Target Position", liftPosition)

    }
}

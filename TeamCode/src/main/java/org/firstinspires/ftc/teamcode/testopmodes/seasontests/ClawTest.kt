package org.firstinspires.ftc.teamcode.testopmodes.seasontests

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot

@TeleOp(name="Claw Test", group="Test")
class ClawTest: OpMode() {

    lateinit var robot: ExtThinBot
    lateinit var gamepad1Ex: GamepadEx
    lateinit var gamepad2Ex: GamepadEx

    private lateinit var toggleGamepad1TouchpadPress: ToggleButtonWatcher
    private lateinit var toggleGamepad2TouchpadPress: ToggleButtonWatcher

    var clawServoPosition: Double = 0.5

    override fun init() {
        robot = ExtThinBot(hardwareMap)

        gamepad1Ex = GamepadEx(gamepad1)
        gamepad2Ex = GamepadEx(gamepad2)

        toggleGamepad1TouchpadPress = ToggleButtonWatcher { gamepad1.touchpad }
        toggleGamepad2TouchpadPress = ToggleButtonWatcher { gamepad2.touchpad }
    }

    override fun loop() {
        gamepad1Ex.readButtons()
        gamepad2Ex.readButtons()
        toggleGamepad1TouchpadPress()
        toggleGamepad2TouchpadPress()

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            clawServoPosition += 0.01
        }
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            clawServoPosition -= 0.01
        }

        robot.clawServo.position = clawServoPosition

        telemetry.addData("Claw Servo Position", clawServoPosition)

    }
}
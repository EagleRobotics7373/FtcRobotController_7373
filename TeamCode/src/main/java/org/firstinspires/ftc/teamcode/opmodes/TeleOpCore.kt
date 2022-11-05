package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.meet1.LiftClawSystem
import kotlin.math.absoluteValue
import kotlin.math.pow

@TeleOp(name="TeleOpCore (Kotlin)", group ="A")
class TeleOpCore: OpMode() {

    lateinit var robot: ExtThinBot
    lateinit var gamepad1Ex: GamepadEx
    lateinit var gamepad2Ex: GamepadEx

    private var reverse = false/* by DashboardVar(false, "reverse", this::class)*/
    private var speed = 3/*by DashboardVar(1, "speed", this::class) {it in 1..3}*/
    private var speedMax: Double = 5.0
    private var maxRpm = 435
    private var cubicEnable = false
    private var fod = false
    private var zeroAngle = 0.0
    private var lastTimeRead = 0.0

    private var liftPowerAuto1 = -0.8
    private var liftPowerAuto2 = -0.8

    private lateinit var elapsedTime: ElapsedTime

    private val orientation get() = robot.imuControllerC.imuA.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS)

    private val gamepad1CanControlAccessories: Boolean
        get() = false //gamepad1.left_bumper && gamepad1.right_bumper

    private val gamepad2CanControlExtras: Boolean
        get() = false //gamepad2.right_bumper

    private lateinit var toggleGamepad1TouchpadPress: ToggleButtonWatcher
    private lateinit var toggleGamepad2TouchpadPress: ToggleButtonWatcher

    override fun init() {
        robot = ExtThinBot(hardwareMap)
        robot.holonomic.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        robot.linearActuatorMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        gamepad1Ex = GamepadEx(gamepad1)
        gamepad2Ex = GamepadEx(gamepad2)
        elapsedTime = ElapsedTime()

        toggleGamepad1TouchpadPress = ToggleButtonWatcher { gamepad1.touchpad }
        toggleGamepad2TouchpadPress = ToggleButtonWatcher { gamepad2.touchpad }
    }

    override fun loop() {

        gamepad1Ex.readButtons()
        gamepad2Ex.readButtons()
        toggleGamepad1TouchpadPress()
        toggleGamepad2TouchpadPress()

        if (!gamepad1CanControlAccessories) {
            // Adjust drivetrain speed and cubic enable
            when {
                gamepad1Ex.wasJustPressed(DPAD_UP) -> if (speed < speedMax) speed++
                gamepad1Ex.wasJustPressed(DPAD_DOWN) -> if (speed > 1) speed--
                gamepad1Ex.wasJustPressed(DPAD_LEFT) -> cubicEnable = !cubicEnable
            }

            // Reverse drivetrain and fod enable
            when {
                gamepad1Ex.wasJustPressed(Y) -> { reverse = !reverse; fod = false }
                gamepad1Ex.wasJustPressed(X) -> fod = true
                gamepad1.x && gamepad1.start -> zeroAngle = robot.imuControllerC.getHeading()
            }
        }

        val vertical = -gamepad1.left_stick_y.toDouble().pow(if (cubicEnable) 3 else 1) * (speed/speedMax) * (if (reverse) 1 else -1)
        val horizontal = gamepad1.left_stick_x.toDouble().pow(if (cubicEnable) 3 else 1) * (speed/speedMax) * (if (reverse) 1 else -1)
        val pivot = gamepad1.right_stick_x.toDouble().pow(if (cubicEnable) 3 else 1) * (speed/speedMax)

        robot.holonomic.runWithoutEncoderVectored(horizontal, vertical, pivot,
                if (fod) zeroAngle - robot.imuControllerC.getHeading() else 0.0)


        // Cycle lift positions
        when {
            gamepad2Ex.wasJustPressed(RIGHT_BUMPER) -> robot.liftClawSystem.liftCycleUp(liftPowerAuto1, liftPowerAuto2)
            gamepad2Ex.wasJustPressed(LEFT_BUMPER) -> robot.liftClawSystem.liftCycleDown(-liftPowerAuto1/8, -liftPowerAuto2)
            gamepad2Ex.wasJustPressed(X) -> robot.liftClawSystem.liftAuto(LiftClawSystem.LiftPosition.FLOOR.ticks1, LiftClawSystem.LiftPosition.FLOOR.ticks2, liftPowerAuto1, liftPowerAuto2)
        }

        // Manual lift control
        var linearActuatorPower = if (!gamepad2CanControlExtras) gamepad2.left_stick_y.toDouble()*0.5 else 0.0
        if (linearActuatorPower in -0.2..0.0 && robot.linearActuatorMotor.currentPosition < -100) linearActuatorPower = -0.2
        else if (linearActuatorPower in 0.01..1.0) linearActuatorPower = 0.0
        val liftPower = if (!gamepad2CanControlExtras) -gamepad2.right_stick_y.toDouble() else 0.0
        if (robot.linearActuatorMotor.mode == DcMotor.RunMode.RUN_TO_POSITION) {
            if (gamepad2.left_stick_y.absoluteValue > 0 ) robot.liftClawSystem.liftManual(0.0, 0.0)
        } else if (gamepad2.y && gamepad2CanControlExtras) {
            robot.liftClawSystem.liftManual(-0.10, 0.0)
        } else {
            robot.liftClawSystem.liftManual(linearActuatorPower, liftPower)
        }

        when {
            gamepad2.a -> robot.liftClawSystem.clawOpen()
            gamepad2.b -> robot.liftClawSystem.clawClose()
        }

        // Recalibrate Lift Bottom Position
        if (gamepad2Ex.wasJustPressed(Y)) {
            robot.linearActuatorMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

        val currentTime = elapsedTime.milliseconds()
        telemetry.addData("Time Δ (ms)", currentTime - lastTimeRead)
        telemetry.addLine()
        lastTimeRead = currentTime
        telemetry.addData("Drivetrain speed", "$speed out of $speedMax")
        telemetry.addData("Drivetrain speed adj", speed/speedMax)
        telemetry.addData("Drivetrain max rpm", maxRpm * (speed/speedMax))
        telemetry.addData("Cubic enable", cubicEnable)
        telemetry.addLine()
        telemetry.addData("Linear actuator power", linearActuatorPower)
        telemetry.addData("Lift power", liftPower)
        telemetry.addData("Linear actuator motor position", robot.linearActuatorMotor.currentPosition)
        telemetry.addData("Lift motor position", robot.liftMotor.currentPosition)
        telemetry.addLine()
        telemetry.addData("Vertical", vertical)
        telemetry.addData("Horizontal", horizontal)
        telemetry.addData("Pivot", pivot)
        telemetry.addLine()
        telemetry.addData("frontRightMotor", robot.frontRightMotor.power)
        telemetry.addData("backRightMotor", robot.backRightMotor.power)
        telemetry.addData("frontLeftMotor", robot.frontLeftMotor.power)
        telemetry.addData("backLeftMotor", robot.backLeftMotor.power)
        telemetry.addLine()
        telemetry.addData("gamepad2.right_trigger", gamepad2.right_trigger)
        telemetry.addData("gamepad2.left_trigger", gamepad2.left_trigger)
        telemetry.addData("gamepad1.right_trigger", gamepad1.right_trigger)
        telemetry.addData("gamepad1.left_trigger", gamepad1.left_trigger)
        telemetry.addLine()
        telemetry.addData("Z Orientation", orientation.firstAngle)
        telemetry.addData("Y Orientation", orientation.secondAngle)
        telemetry.addData("X Orientation", orientation.thirdAngle)
        telemetry.addLine()
        telemetry.addData("Gamepad1 Touchpad Press", gamepad1.touchpad)
        telemetry.addData("Gamepad1 Finger1 X", gamepad1.touchpad_finger_1_x)
        telemetry.addData("Gamepad1 Finger1 Y", gamepad1.touchpad_finger_1_y)
        telemetry.addData("Gamepad1 Finger 1 Press", gamepad1.touchpad_finger_1)
        telemetry.addData("Gamepad1 Finger2 X", gamepad1.touchpad_finger_2_x)
        telemetry.addData("Gamepad1 Finger2 Y", gamepad1.touchpad_finger_2_y)
        telemetry.addData("Gamepad1 Finger 2 Press", gamepad1.touchpad_finger_2)
        telemetry.addData("Gamepad2 Touchpad Press", gamepad2.touchpad)
        telemetry.addData("Gamepad2 Finger1 X", gamepad2.touchpad_finger_1_x)
        telemetry.addData("Gamepad2 Finger1 Y", gamepad2.touchpad_finger_1_y)
        telemetry.addData("Gamepad1 Finger 1 Press", gamepad2.touchpad_finger_1)
        telemetry.addData("Gamepad2 Finger2 X", gamepad2.touchpad_finger_2_x)
        telemetry.addData("Gamepad2 Finger2 Y", gamepad2.touchpad_finger_2_y)
        telemetry.addData("Gamepad1 Finger 2 Press", gamepad2.touchpad_finger_2)
    }
}
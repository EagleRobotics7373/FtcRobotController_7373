package org.firstinspires.ftc.teamcode.testopmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.ReflectiveMenuItemEnum
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.ReflectiveMenuItemFeedback
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.ReflectiveTelemetryMenu

@TeleOp
class MotorEncoderTest : OpMode() {
    lateinit var menu: ReflectiveTelemetryMenu
    lateinit var motor: DcMotorEx
    lateinit var motors : Array<DcMotorEx>

    lateinit var dpadUpWatch : ToggleButtonWatcher
    lateinit var dpadDownWatch : ToggleButtonWatcher
    lateinit var dpadLeftWatch : ToggleButtonWatcher
    lateinit var dpadRightWatch : ToggleButtonWatcher

    override fun init() {
        motors = hardwareMap.getAll(DcMotorEx::class.java).toTypedArray()
        if (motors.isNullOrEmpty()) throw(IllegalStateException("There are no registered DcMotorEx devices!"))
        motor = motors.first()

        for (m in motors) {
            m.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

        menu = ReflectiveTelemetryMenu(
                telemetry,
                ReflectiveMenuItemEnum("motor", ::motor, *motors, toStringMethod = {hardwareMap.getNamesOf(it).first()}),
                ReflectiveMenuItemFeedback("motor position") {motor.currentPosition.toString()},
                ReflectiveMenuItemFeedback("motor velocity") {motor.velocity.toString()}
        )

        dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}
    }

    override fun loop() {
        when {
            dpadUpWatch.invoke()    -> menu.previousItem()
            dpadDownWatch.invoke()  -> menu.nextItem()
            dpadLeftWatch.invoke()  -> menu.iterateBackward()
            dpadRightWatch.invoke() -> menu.iterateForward()
            else -> menu.refresh()
        }
    }

}
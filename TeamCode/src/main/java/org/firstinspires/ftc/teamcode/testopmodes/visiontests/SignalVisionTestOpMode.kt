package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.rangeClip
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.powerplay.SignalVisionConstants
import org.firstinspires.ftc.teamcode.library.vision.powerplay.SignalVisionPipeline

@TeleOp(name="OpenCV: SignalVisionTest", group="Vision")
class SignalVisionTestOpMode: LinearOpMode() {

    var useStandardized = false

    override fun runOpMode() {

        FtcDashboard.getInstance().addConfigVariable(
            this::class.simpleName,
            "Standardized?",
            object: ValueProvider<Boolean> {
                override fun get(): Boolean = useStandardized

                override fun set(value: Boolean?) {
                    if (value != null) {
                        useStandardized = value
                    }
                }
            },
        true)

        val gamepad1Ex = GamepadEx(gamepad1)

        val cvContainer = VisionFactory.createOpenCv(
            hardwareMap,
            "Webcam 1",
            SignalVisionPipeline())
        cvContainer.start()

        var index = 0

        waitForStart()

        cvContainer.pipeline.tracking = true
        cvContainer.pipeline.shouldKeepTracking = true

        while (opModeIsActive()) {
            gamepad1Ex.readButtons()
            if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                index = when (index) {
                    in 0..1 -> index + 1
                    else -> 0
                }
            }
            if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                index = when (index) {
                    in 1..2 -> index - 1
                    else -> 2
                }
            }

            val contourResults = arrayOf(
                    cvContainer.pipeline.contourResult1?.let {
                if (useStandardized) it.standardized else it
            },
                    cvContainer.pipeline.contourResult2?.let {
                if (useStandardized) it.standardized else it
            },
                    cvContainer.pipeline.contourResult3?.let {
                if (useStandardized) it.standardized else it
            }
            )
            val contourResult = contourResults[index]

            telemetry.addData("Standardized", useStandardized)
            telemetry.addLine()

            if (contourResult == null) {
                telemetry.addData("Detected", "No")
            } else {
                telemetry.addData("Detected", "Yes")
                telemetry.addData("Min", contourResult.min.toString())
                telemetry.addData("Max", contourResult.min.toString())
                telemetry.addData("Area", contourResult.area.toString())
                telemetry.addData("Ratio", contourResult.ratio.toString())
                telemetry.addData("Width", contourResult.width.toString())

                val centerX = (contourResult.min.x + contourResult.max.x)/2
                telemetry.addData("Center X", centerX)

                if (useStandardized) {
                    telemetry.addData("Field position (by center x)", when (centerX) {
                        in 0.0..SignalVisionConstants.BOUNDARY_FIRST -> "LEFT"
                        in SignalVisionConstants.BOUNDARY_FIRST..SignalVisionConstants.BOUNDARY_SECOND -> "CENTER"
                        else -> "RIGHT"
                    })
                }
            }

            when {
                gamepad1.a -> useStandardized = true
                gamepad1.b -> useStandardized = false
            }

            telemetry.update()
        }

    }
}
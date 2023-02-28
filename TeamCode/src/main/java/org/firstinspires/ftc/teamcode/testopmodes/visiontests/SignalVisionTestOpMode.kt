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
    var index = 0

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

        FtcDashboard.getInstance().addConfigVariable(
                this::class.simpleName,
                "index",
                object: ValueProvider<Int> {
                    override fun get(): Int = index

                    override fun set(value: Int?) {
                        if (value != null) {
                            index = value
                        }
                    }
                }
        )

        val gamepad1Ex = GamepadEx(gamepad1)

        val cvContainer = VisionFactory.createOpenCv(
            hardwareMap,
            "Webcam 1",
            SignalVisionPipeline())
        cvContainer.start()

        waitForStart()

        cvContainer.pipeline.tracking = true
        cvContainer.pipeline.shouldKeepTracking = true

        while (opModeIsActive()) {
            gamepad1Ex.readButtons()

            val contourResults = cvContainer.pipeline.contourResults

            for (i in contourResults.indices){
                telemetry.addData("Index", i)
                telemetry.addLine()

                if (contourResults[i] == null) {
                    telemetry.addData("\tDetected", "No")
                    telemetry.addLine()
                } else {
                    telemetry.addData("\tDetected", "Yes")
                    telemetry.addData("\tMin", contourResults[i]!!.min.toString())
                    telemetry.addData("\tMax", contourResults[i]!!.min.toString())
                    telemetry.addData("\tArea", contourResults[i]!!.area.toString())
                    telemetry.addData("\tRatio", contourResults[i]!!.ratio.toString())
                    telemetry.addData("\tWidth", contourResults[i]!!.width.toString())
                    telemetry.addData("\tValid", contourResults[i]!!.valid.toString())

                    val centerX = (contourResults[i]!!.min.x + contourResults[i]!!.max.x) / 2
                    telemetry.addData("\tCenter X", centerX)
                    telemetry.addLine()

                }
            }

            telemetry.update()
        }

    }
}
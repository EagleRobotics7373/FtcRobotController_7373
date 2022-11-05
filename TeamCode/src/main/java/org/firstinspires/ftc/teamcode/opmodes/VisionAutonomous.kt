package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.SignalState
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.powerplay.SignalVisionPipeline

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision Autonomous", group = "Main")
class VisionAutonomous : BaseAutonomous<ExtThinBot>() {

    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var startingPosition: StartingPosition by config.custom("Starting Position", LEFT, RIGHT)
    private var signalState: SignalState by config.custom("Default Parking Position", SignalState.CENTER, SignalState.LEFT, SignalState.RIGHT)
    private var extraDelayBeforeStart: Int by config.int("Delay Before First Action", 0, 0..20000 step 1000)
    private var webcamScanningDuration: Int by config.int("Webcam Scanning Duration", 2000, 0..5000 step 500)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
        super.autonomousConfig()

        val cvContainer = VisionFactory.createOpenCv(
                hardwareMap,
                "Webcam 1",
                SignalVisionPipeline())
        cvContainer.start()

        super.operateMenu(null)

        if (opModeIsActive()) {
            cvContainer.pipeline.tracking = true
            cvContainer.pipeline.shouldKeepTracking = true

            sleep(webcamScanningDuration.toLong())

            val contourResults = cvContainer.pipeline.contourResults

            signalState = if (contourResults[1]?.valid == true) SignalState.CENTER
            else if (contourResults[0]?.valid == true) SignalState.LEFT
            else if (contourResults[2]?.valid == true) SignalState.RIGHT
            else signalState

            telem.addData("Parking Position for $allianceColor Alliance", signalState)
            telem.update()

            sleep(extraDelayBeforeStart.toLong())

            robot.holonomic.runWithoutEncoderVectored(
                    when (signalState) {
                        SignalState.LEFT -> 0.75
                        SignalState.RIGHT -> -0.75
                        else -> 0.0 },
                    0.0, 0.0, 0.0
            )
            sleep(1000L)
            robot.holonomic.stop()

            robot.holonomic.runWithoutEncoderVectored(0.0, -0.75, 0.0, 0.0)
            sleep(700L)
            robot.holonomic.stop()
        }

    }


}
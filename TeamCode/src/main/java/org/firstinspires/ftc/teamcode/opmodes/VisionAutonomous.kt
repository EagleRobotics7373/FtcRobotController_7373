package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.SignalState
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.powerplay.SignalVisionPipeline

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision Autonomous (Broken)", group = "Main")
class VisionAutonomous : BaseAutonomous<ExtThinBot>() {

    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var startingPosition: StartingPosition by config.custom("Starting Position", LEFT, RIGHT)
    private var signalState: SignalState by config.custom("Default Parking Position", SignalState.LEFT, SignalState.CENTER, SignalState.RIGHT)
    private var extraDelayBeforeStart: Int by config.int("Delay Before First Action", 0, 0..20000 step 1000)
    private var webcamScanningDuration: Int by config.int("Webcam Scanning Duration", 2000, 0..5000 step 500)
    private var safeModeActive: Boolean by config.boolean("Safe Mode", true)
    private var safeModeErrorThreshold: Int by config.int("Safe Mode Error Threshold", 10, 0..30 step 2)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
        super.autonomousConfig()
        robot.holonomicRR.safeModeErrorThreshold = safeModeErrorThreshold

        val cvContainer = VisionFactory.createOpenCv(
                hardwareMap,
                "Webcam 1",
                SignalVisionPipeline())
        cvContainer.start()

        super.operateMenu(null)

        if (opModeIsActive()) {
            cvContainer.pipeline.tracking = true
            cvContainer.pipeline.shouldKeepTracking = true

            robot.holonomicRR.poseEstimate = Pose2d(
                    when (startingPosition) {
                        LEFT -> -36.0
                        RIGHT -> 36.0
                    } reverseIf BLUE,
                    -63.0 reverseIf BLUE,
                    Math.PI / 2 reverseIf BLUE
            )

            sleep(webcamScanningDuration.toLong())

            val contourResults = arrayOf(
                    cvContainer.pipeline.contourResult1,
                    cvContainer.pipeline.contourResult2,
                    cvContainer.pipeline.contourResult3
            )

            signalState = if (contourResults[1] != null) SignalState.CENTER
            else if (contourResults[0] != null) SignalState.LEFT
            else if (contourResults[2] != null) SignalState.RIGHT
            else signalState

            telem.addData("Parking Position for $allianceColor Alliance", signalState)
            telem.update()

            sleep(extraDelayBeforeStart.toLong())

            if (signalState != SignalState.CENTER) {
                builder().strafeTo(
                        Vector2d(
                                when (startingPosition) {
                                    LEFT -> -36.0
                                    RIGHT -> 36.0
                                } + signalState.shift reverseIf BLUE,
                                -63.0 reverseIf BLUE
                        )
                ).buildAndRun(safeMode = safeModeActive)
            }
            safeModeCheck(safeModeActive)

            builder().strafeTo(
                    Vector2d(
                            when (startingPosition) {
                                LEFT -> -36.0
                                RIGHT -> 36.0
                            } + signalState.shift reverseIf BLUE,
                            -36.0 reverseIf BLUE
                    )
            ).buildAndRun(safeMode = safeModeActive)
            safeModeCheck(safeModeActive)
        }

    }

    private infix fun Double.reverseIf(testColor: AllianceColor) : Double = if (allianceColor == testColor) -this else this

}
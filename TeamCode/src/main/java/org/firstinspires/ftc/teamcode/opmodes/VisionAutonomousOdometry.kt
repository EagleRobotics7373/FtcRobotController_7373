package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.BLUE
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.RED
import org.firstinspires.ftc.teamcode.library.functions.SignalState
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition.LEFT
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition.RIGHT
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.lt.DualServoClawLift
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.powerplay.SignalVisionPipeline

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision Autonomous (Odometry)", group = "Main")
class VisionAutonomousOdometry : BaseAutonomous<ExtThinBot>() {

    private var startingPosition: StartingPosition by config.custom("Starting Position", LEFT, RIGHT)
    private var coneStack: Int by config.int("Cone Stack", 1, 1..4 step 1)
    private var signalState: SignalState by config.custom("Default Parking Position", SignalState.CENTER, SignalState.LEFT, SignalState.RIGHT)
    private var extraDelayBeforeStart: Int by config.int("Delay Before First Action", 0, 0..20000 step 1000)
    private var webcamScanningDuration: Int by config.int("Webcam Scanning Duration", 2000, 0..5000 step 500)
    private var safeModeActive: Boolean by config.boolean("Safe Mode", false)
    private var safeModeErrorThreshold: Int by config.int("Safe Mode Error Threshold", 10, 0..30 step 2)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
        super.autonomousConfig()
        robot.holonomicRR.safeModeErrorThreshold = safeModeErrorThreshold

        robot.linearActuatorMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

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
                    36.0 reverseIf LEFT,
                    63.0,
                    Math.PI / 2
            )

            sleep(webcamScanningDuration.toLong())

            // Detect Signal
            val contourResults = cvContainer.pipeline.contourResults
            if (contourResults[1]?.valid == true) {
                signalState = SignalState.CENTER
                println("Succeeded to Detect Signal")
            }
            else if (contourResults[0]?.valid == true) {
                signalState = SignalState.LEFT
                println("Succeeded to Detect Signal")
            }
            else if (contourResults[2]?.valid == true) {
                signalState = SignalState.RIGHT
                println("Succeeded to Detect Signal")
            }
            else {
                telem.addLine("Failed to Detect Signal, Using Default Value")
                println("Failed to Detect Signal, Using Default Value")
            }

            telem.addData("Parking Position", signalState)
            telem.update()

            sleep(extraDelayBeforeStart.toLong())

            //Pick up Cone
            robot.dualServoClawLift.clawClose()
            sleep(1000)
            robot.dualServoClawLift.liftAuto(DualServoClawLift.LiftPosition.MIDDLE, -0.85)

            //Drive to Goal
            builder().back(35.0).buildAndRun(safeModeActive)
            safeModeCheck(safeModeActive)
            sleep(500)
            builder().strafeLeft(12.0 reverseIf LEFT).buildAndRun(safeModeActive)
            safeModeCheck(safeModeActive)
            sleep(250)

            //Drop Cone
            robot.dualServoClawLift.liftAuto(DualServoClawLift.LiftPosition.LOW, -0.85)
            robot.dualServoClawLift.clawOpen()
            sleep(250)

            //Back up
            builder().forward(4.0).buildAndRun(safeModeActive)

            //Drive to Parking
            if (startingPosition == RIGHT) {
                when (signalState) {
                    SignalState.LEFT -> builder().strafeLeft(12.0).buildAndRun(safeModeActive)
                    SignalState.CENTER -> builder().strafeLeft(-12.0).buildAndRun(safeModeActive)
                    SignalState.RIGHT -> builder().strafeLeft(-26.0).buildAndRun(safeModeActive)
                }
            } else {
                when (signalState) {
                    SignalState.RIGHT -> builder().strafeLeft(-12.0).buildAndRun(safeModeActive)
                    SignalState.CENTER -> builder().strafeLeft(12.0).buildAndRun(safeModeActive)
                    SignalState.LEFT -> builder().strafeLeft(28.0).buildAndRun(safeModeActive)
                }
            }

        }

    }

    private infix fun Double.reverseIf(testPosition: StartingPosition) : Double = if (startingPosition == testPosition) -this else this

}
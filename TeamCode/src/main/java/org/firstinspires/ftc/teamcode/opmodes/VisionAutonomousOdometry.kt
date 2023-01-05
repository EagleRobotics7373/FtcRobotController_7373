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
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.powerplay.SignalVisionPipeline

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision Autonomous (Odometry)", group = "Main")
class VisionAutonomousOdometry : BaseAutonomous<ExtThinBot>() {

    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var startingPosition: StartingPosition by config.custom("Starting Position", LEFT, RIGHT)
    private var signalState: SignalState by config.custom("Default Parking Position", SignalState.CENTER, SignalState.LEFT, SignalState.RIGHT)
    private var extraDelayBeforeStart: Int by config.int("Delay Before First Action", 0, 0..20000 step 1000)
    private var webcamScanningDuration: Int by config.int("Webcam Scanning Duration", 2000, 0..5000 step 500)
    private var safeModeActive: Boolean by config.boolean("Safe Mode", true)
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
                    -36.0 reverseIf RIGHT reverseIf BLUE,
                    -63.0 reverseIf BLUE,
                    -Math.PI / 2 reverseIf BLUE
            )

            sleep(webcamScanningDuration.toLong())

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

            telem.addData("Parking Position for $allianceColor Alliance", signalState)
            telem.update()

            sleep(extraDelayBeforeStart.toLong())

            //Drive to Cone
            builder().strafeTo(
                    Vector2d(
                            -36.0 reverseIf RIGHT
                                    reverseIf BLUE, -10.5 reverseIf BLUE
                    )
            ).buildAndRun(safeModeActive)
            safeModeCheck(safeModeActive)

            robot.dualServoClawLift.liftAuto(-290, -0.8)
            robot.dualServoClawLift.clawOpen()

            builder().lineToSplineHeading(
                    Pose2d(
                            -50.6 reverseIf RIGHT
                                    reverseIf BLUE, -12.5 reverseIf BLUE,
                            (Math.PI / 2 reverseIf BLUE) + (Math.PI / 2 reverseIf RIGHT)
                    )
            ).buildAndRun()

            //Pick Up Cone
            robot.dualServoClawLift.clawClose()
            sleep(1000)
            robot.dualServoClawLift.liftAuto(-1000, 0.3)

            builder().lineToSplineHeading(
                    Pose2d(
                            -22.0 reverseIf RIGHT
                                    reverseIf BLUE, -17.0 reverseIf BLUE,
                            -Math.PI / 2 reverseIf BLUE
                    ),
                    DriveConstraints(
                            10.0, 25.0, 40.0,
                            Math.PI, Math.PI, 0.0
                    )
            ).buildAndRun()

            //Drop Off Cone
            sleep(100)
            robot.dualServoClawLift.clawOpen()

            // Park
            builder().strafeTo(
                    Vector2d(
                            ((-36.0 reverseIf RIGHT) + signalState.shift) reverseIf BLUE,
                            -12.5 reverseIf BLUE
                    )
            ).buildAndRun()
            builder().strafeTo(
                    Vector2d(
                            ((-36.0 reverseIf RIGHT) + signalState.shift) reverseIf BLUE,
                            -36.0 reverseIf BLUE
                    )
            ).buildAndRun()

            robot.dualServoClawLift.liftAuto(0, 0.01)
            sleep(1000)
        }

    }

    private infix fun Double.reverseIf(testColor: AllianceColor) : Double = if (allianceColor == testColor) -this else this
    private infix fun Double.reverseIf(testPosition: StartingPosition) : Double = if (startingPosition == testPosition) -this else this

}
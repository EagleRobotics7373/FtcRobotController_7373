package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsThinBot.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem.DepositPosition
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem.DepositPosition.*
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionPipeline

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision Autonomous", group = "Main")
class CarouselVisionAutonomous : BaseAutonomous<ExtThinBot>() {

    /*
        VARIABLES: Hardware and Control
     */

    /*
        VARIABLES: Menu Options
     */
    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var depositPosition: DepositPosition by config.custom("Deposit Position", LOW, MIDDLE, HIGH)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
        val cvContainer = VisionFactory.createOpenCv(
                hardwareMap,
                "Webcam 1",
                ColorMarkerVisionPipeline())
        cvContainer.start()

        waitForStart()

        cvContainer.pipeline.tracking = true
        cvContainer.pipeline.shouldKeepTracking = true
        super.operateMenu()

        while (opModeIsActive()) {
            val contourResult = cvContainer.pipeline.contourResult?.standardized
            depositPosition = if (contourResult != null) {
                val center = (contourResult.max.x + contourResult.min.x) / 2
                if (center < 0.33) LOW
                else if (center < 0.66) MIDDLE
                else HIGH
            } else HIGH

            robot.holonomicRR.poseEstimate = Pose2d(
                    -36.0,
                    (-63.0).reverseIf(allianceColor == BLUE),
                    (Math.PI / 2).reverseIf(allianceColor == BLUE) //startingHeading
            )

            builder()
                    .strafeTo(Vector2d(-14.5, (-46.9).reverseIf(allianceColor == BLUE)))
                    .splineToConstantHeading(Vector2d(-12.5, (-43.1).reverseIf(allianceColor == BLUE)), (Math.PI / 2).reverseIf(allianceColor == BLUE))
                    .buildAndRun()

            //Drop Off Pre-Load
            robot.fullIntakeSystem.depositLiftAuto(depositPosition, 1.0)
            robot.depositServo.position = 0.32

            builder()
                    .strafeTo(Vector2d(-63.0, (-53.0).reverseIf(allianceColor == BLUE)))
                    .buildAndRun()

            //Turn Carousel
            robot.carouselMotor.velocity = (0.5).reverseIf(allianceColor == BLUE)
            sleep(1000)
            robot.carouselMotor.velocity = 0.0

            builder()
                    .strafeTo(Vector2d(-63.0, (-53.0).reverseIf(allianceColor == BLUE)))
                    .buildAndRun()
        }
    }

    private infix fun Double.reverseIf(testColor: AllianceColor) : Double = if (allianceColor == testColor) -this else this
}

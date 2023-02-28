package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.library.functions.SignalState
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition.LEFT
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition.RIGHT
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.lt.DualServoClawLift
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.powerplay.SignalVisionPipeline
import kotlin.math.abs

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision Autonomous", group = "Main")
class VisionAutonomous : BaseAutonomous<ExtThinBot>() {

    private var startingPosition: StartingPosition by config.custom("Starting Position", LEFT, RIGHT)
    private var coneStack: Int by config.int("Cone Stack", 1, 1..6 step 1)
    private var mediumGoal: Boolean by config.boolean("Medium Target", true)
    private var signalState: SignalState by config.custom("Default Parking Position", SignalState.CENTER, SignalState.LEFT, SignalState.RIGHT)
    private var adjustForVoltage: Boolean by config.boolean("Adjust for Voltage", true)
    private var extraDelayBeforeStart: Int by config.int("Delay Before First Action", 0, 0..20000 step 1000)
    private var webcamScanningDuration: Int by config.int("Webcam Scanning Duration", 1000, 0..5000 step 500)
    private var safeModeActive: Boolean by config.boolean("Safe Mode", false)
    private var safeModeErrorThreshold: Int by config.int("Safe Mode Error Threshold", 10, 0..30 step 2)
    private val desiredVoltage = 13.7

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
        super.autonomousConfig()
        robot.holonomicRR.safeModeErrorThreshold = safeModeErrorThreshold

        val currentVoltage = if (adjustForVoltage) robot.controlHub.getInputVoltage(VoltageUnit.VOLTS) else desiredVoltage
        val voltageAdjustment = if (adjustForVoltage) desiredVoltage/currentVoltage else 1.0
        val rotationVoltageAdjustment = (voltageAdjustment - 1)*abs(voltageAdjustment - 1) + 1

        robot.linearActuatorMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        val cvContainer = VisionFactory.createOpenCv(
                hardwareMap,
                "Webcam 1",
                SignalVisionPipeline())
        cvContainer.start()

        super.operateMenu(null)

        val target = if (mediumGoal) {
             DualServoClawLift.LiftPosition.MIDDLE
        } else { DualServoClawLift.LiftPosition.HIGH }
        val targetDistance = if (mediumGoal) {
            42.5
        } else { 66.5 }

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

            cvContainer.pipeline.tracking = false
            cvContainer.pipeline.shouldKeepTracking = false

            telem.addData("Parking Position", signalState)
            telem.update()

            sleep(extraDelayBeforeStart.toLong())

            //Pick up Cone
            robot.dualServoClawLift.clawClose()
            sleep(1000)
            robot.dualServoClawLift.liftAuto(target, -0.85)

            //Drive to Goal (first time)
            builder().back(targetDistance * voltageAdjustment).buildAndRun(safeModeActive)
            builder().back(-11.0 * voltageAdjustment).buildAndRun(safeModeActive)
            safeModeCheck(safeModeActive)
            robot.holonomicRR.turnSync((Math.PI / 4 + ((-0.025) ifLeft 0.1)) * rotationVoltageAdjustment reverseIf LEFT)
            builder().back((10.75 ifLeft 4.5) * voltageAdjustment).buildAndRun(safeModeActive)
            sleep(500)

            //Drop Cone
            robot.dualServoClawLift.clawOpen()
            //robot.dualServoClawLift.liftAuto(-450, -0.85, true)
            sleep(100)

            //Back up
            builder().back(-11.5 * voltageAdjustment).buildAndRun(safeModeActive)

            if (coneStack > 1)  {
                for (i in 1 until coneStack) {
                    //Turn and Drive to Cone
                    robot.holonomicRR.turnSync((-3*Math.PI / 4 - (0.075 ifLeft 0.1)) * rotationVoltageAdjustment reverseIf LEFT)
                    sleep(250)
                    val distance = robot.distanceSystem.getFrontDistance(expectedValue = 22.0)
                    telem.addData("Distance", distance)
                    telem.update()
                    builder().back((distance + (1.0 ifLeft 2.0)) * voltageAdjustment).buildAndRun(safeModeActive)
                    sleep(250)

                    //Pick up Cone
                    robot.dualServoClawLift.clawClose()
                    sleep(750)
                    robot.dualServoClawLift.liftAuto(DualServoClawLift.LiftPosition.HIGH, -0.85)
                    sleep(250)

                    //Drive to Goal
                    builder().back((-distance - (1.5 ifLeft 5.0)) * voltageAdjustment).buildAndRun(safeModeActive)
                    robot.holonomicRR.turnSync((3*Math.PI / 4 + (0.075 ifLeft 0.075)) * rotationVoltageAdjustment reverseIf LEFT)
                    builder().back((15.0 ifLeft 10.5) * voltageAdjustment).buildAndRun(safeModeActive)

                    //Drop Cone
                    robot.dualServoClawLift.clawOpen()
                    robot.dualServoClawLift.liftAuto(-450, -0.85, true)
                    sleep(100)

                    //Back up
                    builder().back(-14.0 * voltageAdjustment).buildAndRun(safeModeActive)
                }
            }

            //Drive to Parking
            robot.holonomicRR.turnSync((-Math.PI / 4) reverseIf LEFT)
            sleep(250)
            if (startingPosition == RIGHT) {
                val sideDistance = robot.distanceSystem.getRightDistance()
                telem.addData("Distance", sideDistance)
                telem.update()
                when (signalState) {
                    SignalState.LEFT -> builder().strafeLeft(-sideDistance + 43.5).buildAndRun(safeModeActive)
                    SignalState.RIGHT -> builder().strafeLeft(-sideDistance + 10.5).buildAndRun(safeModeActive)
                    SignalState.CENTER -> builder().strafeLeft(-sideDistance + 30.0).buildAndRun(safeModeActive)
                }
            } else {
                val sideDistance = robot.distanceSystem.getLeftDistance()
                telem.addData("Distance", sideDistance)
                telem.update()
                when (signalState) {
                    SignalState.RIGHT -> builder().strafeLeft(sideDistance - 44.5).buildAndRun(safeModeActive)
                    SignalState.LEFT -> builder().strafeLeft(sideDistance - 11.5).buildAndRun(safeModeActive)
                    SignalState.CENTER -> builder().strafeLeft(sideDistance - 30.0).buildAndRun(safeModeActive)
                }
            }
        }
    }

    private infix fun Double.reverseIf(testPosition: StartingPosition) : Double = if (startingPosition == testPosition) -this else this
    private infix fun Double.ifLeft(leftChoice: Double) : Double = if (startingPosition == LEFT) leftChoice else this

}
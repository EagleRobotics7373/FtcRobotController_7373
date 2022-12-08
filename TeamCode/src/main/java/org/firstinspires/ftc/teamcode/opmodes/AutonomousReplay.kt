package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Replay", group = "Z")
class AutonomousReplay : BaseAutonomous<ExtThinBot>() {
    val vertical = arrayOf(0.0)
    val horizontal = arrayOf(0.0)
    val pivot = arrayOf(0.0)
    val dt = arrayOf(0.0)

    private var lastTimeRead = 0.0

    override fun runOpMode() {

        robot = ExtThinBot(hardwareMap)
        super.autonomousConfig()

        waitForStart()

        elapsedTime = ElapsedTime()
        for (i in dt.indices) {
            if (!opModeIsActive()) {
                stop()
            }
            robot.holonomic.runWithoutEncoderVectored(horizontal[i], vertical[i], pivot[i],0.0)
            var currentTime = elapsedTime.milliseconds()
            telemetry.addData("Time Δ (ms)", currentTime - lastTimeRead)
            telemetry.update()
            if (dt[i] - (currentTime - lastTimeRead) > 0) {
                sleep((dt[i] - (currentTime - lastTimeRead)).toLong())
            }
            telemetry.addData("Time added Δ (ms)", dt[i] - (currentTime - lastTimeRead))
            currentTime = elapsedTime.milliseconds()
            lastTimeRead = currentTime
        }
    }
}
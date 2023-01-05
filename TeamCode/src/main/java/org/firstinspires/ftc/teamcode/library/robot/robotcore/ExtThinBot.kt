package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.library.robot.systems.lt.DualServoClawLift
import org.firstinspires.ftc.teamcode.library.robot.systems.meet1.LiftClawSystem

class ExtThinBot(_hardwareMap: HardwareMap): BaseRobot(_hardwareMap) {

    @JvmField val linearActuatorMotor : DcMotorEx = hwInit("linearActuatorMotor")
    @JvmField val leftClawServo : Servo = hwInit("leftClawServo")
    @JvmField val rightClawServo : Servo = hwInit("rightClawServo")
    @JvmField val odometryRight : DcMotorEx = hwInit("odometryRight")
    @JvmField val odometryCenter : DcMotorEx = hwInit("odometryCenter")

    @JvmField val dualServoClawLift = DualServoClawLift(linearActuatorMotor, leftClawServo, rightClawServo)

    @JvmField val imuControllerC = IMUController(hardwareMap, id = 'C')
    override val holonomicRR: HolonomicRR = HolonomicRR(imuControllerC, frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                    TwoWheelOdometryLocalizer(odometryRight, odometryCenter, imuControllerC))

}
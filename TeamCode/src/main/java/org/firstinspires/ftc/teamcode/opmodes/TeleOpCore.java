package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot;
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.HolonomicImpl;

@Config
@TeleOp
public class TeleOpCore extends OpMode {

    private ExtThinBot robot;


    public static double IN = 0.6;
    public static double OUT = 0.32;

    public static double LIN_POS_IN = 1.0;
    public static double LIN_POS_OUT = 0.0;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double carouselRPS = 10;
    double carouselTPS = carouselRPS * 145.1;


    @Override
    public void init(){
        robot = new ExtThinBot(hardwareMap);
//        robot.carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }


    @Override
    public void loop(){
        if(gamepad2.a) robot.outServo.setPosition(IN);
        else if(gamepad2.b) robot.outServo.setPosition(OUT);

        if(gamepad2.left_trigger > 0.05) robot.intakeMotor.setPower(-gamepad2.left_trigger);
        else if(gamepad2.right_trigger > 0.05) robot.intakeMotor.setPower(gamepad2.right_trigger);
        else robot.intakeMotor.setPower(0.0);

        robot.carouselMotor.setPower(gamepad2.left_stick_y);

        robot.linearActuatorMotor.setPower(gamepad2.right_stick_y * 0.20);

        if (gamepad2.x) robot.linearActuatorServo.setPosition(LIN_POS_OUT);
        if (gamepad2.y) robot.linearActuatorServo.setPosition(LIN_POS_IN);

        double vertical;
        double horizontal;
        double pivot;        double speed;

        //Speed Controls
        if (gamepad1.right_bumper)      speed = 1;
        else if (gamepad1.left_bumper)  speed = 0.3;
        else                            speed = 0.5;

        //Mecanum Drive
        vertical = gamepad1.left_stick_y * speed;
        horizontal = -gamepad1.left_stick_x * speed;
        pivot = gamepad1.right_stick_x * speed;

//        robot.holonomic.runWithoutEncoderVectored(horizontal, vertical, pivot, 0);
        robot.frontRightMotor.setPower(pivot - vertical - horizontal);
        robot.backRightMotor.setPower(pivot - vertical + horizontal);
        robot.frontLeftMotor.setPower(pivot + vertical + horizontal);
        robot.backLeftMotor.setPower(pivot + vertical - horizontal);
    }
}

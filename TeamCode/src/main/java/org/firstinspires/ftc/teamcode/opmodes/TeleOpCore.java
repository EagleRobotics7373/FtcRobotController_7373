package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot;

@Config
@TeleOp
public class TeleOpCore extends OpMode {

    private ExtThinBot robot;

    public static double IN = 1;
    public static double OUT = 0;

    @Override
    public void init(){
        robot = new ExtThinBot(hardwareMap);
    }

    @Override
    public void loop(){
        if(gamepad2.a){
            robot.outServo.setPosition(IN);
        }

        if(gamepad2.b){
            robot.outServo.setPosition(OUT);
        }

        if(gamepad2.left_trigger > 0.05){
            robot.intakeMotor.setPower(-gamepad2.left_trigger);
        } else if(gamepad2.right_trigger > 0.05){
            robot.intakeMotor.setPower(gamepad2.right_trigger);
        }

        double vertical;
        double horizontal;
        double pivot;        double speed;

        //Speed Controls
        if (gamepad1.right_bumper)      speed = 1;
        else if (gamepad1.left_bumper)  speed = 0.3;
        else                            speed = 0.5;

        //Mecanum Drive
        vertical = gamepad1.left_stick_y * speed;
        horizontal = gamepad1.left_stick_x * speed;
        pivot = gamepad1.right_stick_x * speed;


        robot.frontRightMotor.setPower(pivot - vertical - horizontal);
        robot.backRightMotor.setPower(pivot - vertical + horizontal);
        robot.frontLeftMotor.setPower(pivot + vertical + horizontal);
        robot.backLeftMotor.setPower(pivot + vertical - horizontal);

    }
}

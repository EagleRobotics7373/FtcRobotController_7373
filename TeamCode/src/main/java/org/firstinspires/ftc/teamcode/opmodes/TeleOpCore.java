package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class TeleOpCore extends OpMode {
    Servo outServo;
    DcMotor intakeMotor;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;

    public static double IN = 1;
    public static double OUT = 0;

    @Override
    public void init(){
        outServo = hardwareMap.get(Servo.class, "outServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");

    }

    @Override
    public void loop(){
        if(gamepad2.a){
            outServo.setPosition(IN);
        }

        if(gamepad2.b){
            outServo.setPosition(OUT);
        }

        if(gamepad2.left_trigger > 0.05){
            intakeMotor.setPower(-gamepad2.left_trigger);
        } else if(gamepad2.right_trigger > 0.05){
            intakeMotor.setPower(gamepad2.right_trigger);
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


        frontRightMotor.setPower(pivot - vertical - horizontal);
        backRightMotor.setPower(pivot - vertical + horizontal);
        frontLeftMotor.setPower(pivot + vertical + horizontal);
        backLeftMotor.setPower(pivot + vertical - horizontal);

    }
}

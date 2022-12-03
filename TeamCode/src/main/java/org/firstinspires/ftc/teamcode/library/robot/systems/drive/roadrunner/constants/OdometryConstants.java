package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OdometryConstants {

    public static double leftXcm = -16.5;
    public static double leftYcm = 0;
    public static double leftAngleDeg = 0.0;

    public static double rightXcm = -16.5;
    public static double rightYcm = 0;
    public static double rightAngleDeg = 0.0;

    public static double rearXcm = 3.8;
    public static double rearYcm = -10.0;
    public static double rearAngleDeg = 90;

    public static boolean reverseOutput = false;

//    public static Pose2d rightOdometryPose = new Pose2d(INCH.fromCm(-2.2), INCH.fromCm(-19.5), 0.0);
//    public static Pose2d rearOdometryPose = new Pose2d(INCH.fromCm(-17.6), INCH.fromCm(0.0), Math.PI / 2);
//    public static Pose2d leftOdometryPose = new Pose2d(INCH.fromCm(-2.2), INCH.fromCm(18.25), 0.0);

}

package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstantsThinBot {
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 12.0;

    public static double MAX_RPM = 435;
    public static double TICKS_PER_REV = 384.5;
    //uncomment for 312rpm motors
//    public static double MAX_RPM = 312;
//    public static double TICKS_PER_REV = 537.7;

    public static double kV = /*0.0035 .00772*/  /* 1.0 / rpmToVelocity(getMaxRpm())*/ 0.006;
    public static double kA = 0.0;
    public static double kStatic = .014;

    public static Pose2d globalPoseEstimate = null;

    public static boolean RUN_USING_ENCODER = true;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(4.35, 0, 0);

    public static DriveConstraints BASE_CONSTRAINTS =
            new DriveConstraints(
//                    50.0, 30.0, 40.0,
                    30.0, 25.0, 40.0,
                    Math.PI, Math.PI, 0.0
            );

    public static PIDCoefficients TRANSLATIONAL_X_PID =
            new PIDCoefficients(1.32, 0.0, 0.4);

    public static PIDCoefficients TRANSLATIONAL_Y_PID =
            new PIDCoefficients(1.75, 0.0, 0.0);

    public static PIDCoefficients HEADING_PID =
            new PIDCoefficients(10.0, 0.5, 0.75);

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
}
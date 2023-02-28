package org.firstinspires.ftc.teamcode.library.vision.powerplay;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SignalVisionConstants {
    // Lower and upper bounds for cv::inRange() function
    public static double CONTOUR_HUE_LOWER_BOUND_RED = 7;
    public static double CONTOUR_HUE_UPPER_BOUND_RED = 28;
    public static double CONTOUR_HUE_LOWER_BOUND_GREEN = 23;
    public static double CONTOUR_HUE_UPPER_BOUND_GREEN = 94;
    public static double CONTOUR_HUE_LOWER_BOUND_BLUE = 90;
    public static double CONTOUR_HUE_UPPER_BOUND_BLUE = 110;

    public static double CONTOUR_SAT_LOWER_BOUND_RED = 63;
    public static double CONTOUR_SAT_UPPER_BOUND_RED = 255;
    public static double CONTOUR_SAT_LOWER_BOUND_GREEN = 50;
    public static double CONTOUR_SAT_UPPER_BOUND_GREEN = 110;
    public static double CONTOUR_SAT_LOWER_BOUND_BLUE = 63;
    public static double CONTOUR_SAT_UPPER_BOUND_BLUE = 255;

    public static double CONTOUR_LUM_LOWER_BOUND_RED = 40;
    public static double CONTOUR_LUM_UPPER_BOUND_RED = 255;
    public static double CONTOUR_LUM_LOWER_BOUND_GREEN = 20;
    public static double CONTOUR_LUM_UPPER_BOUND_GREEN = 255;
    public static double CONTOUR_LUM_LOWER_BOUND_BLUE = 20;
    public static double CONTOUR_LUM_UPPER_BOUND_BLUE = 140;

    public static int CONTOUR_ENTITY_MINWIDTH = 30;
    public static int MAT_OUTPUT_NUM = 5;
    public static double CONTOUR_DILATION_KSIZE = 2.0;

    public static double RATIO_LOWER_BOUND = 0.5;
    public static double RATIO_UPPER_BOUND = 1.0;

    public static double CUTOFF_TOP = 0.10;
    public static double CUTOFF_BOTTOM = 0.60;
    public static double CUTOFF_LEFT = 0.40;
    public static double CUTOFF_RIGHT = 0.80;
}

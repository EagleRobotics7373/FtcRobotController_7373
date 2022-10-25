package org.firstinspires.ftc.teamcode.library.vision.powerplay;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SignalVisionConstants {
    // Lower and upper bounds for cv::inRange() function
    public static double[] CONTOUR_HUE_LOWER_BOUND = new double[]{0,  53,  128};
    public static double[] CONTOUR_HUE_UPPER_BOUND = new double[]{38,  94,  165};
    public static double[] CONTOUR_SAT_LOWER_BOUND = new double[]{63,  63,  63};
    public static double[] CONTOUR_SAT_UPPER_BOUND = new double[]{255, 255, 255};
    public static double[] CONTOUR_LUM_LOWER_BOUND = new double[]{40,  40,  40};
    public static double[] CONTOUR_LUM_UPPER_BOUND = new double[]{255, 255, 255};
    public static int CONTOUR_ENTITY_MINWIDTH = 30;
    public static int MAT_OUTPUT_NUM = 2;
    public static double CONTOUR_DILATION_KSIZE = 2.0;

    public static double BOUNDARY_FIRST = 0.30; // was 0.51
    public static double BOUNDARY_SECOND = 0.51; // was 0.7

    public static double CUTOFF_TOP = 0.20;
    public static double CUTOFF_BOTTOM = 0.80;
}

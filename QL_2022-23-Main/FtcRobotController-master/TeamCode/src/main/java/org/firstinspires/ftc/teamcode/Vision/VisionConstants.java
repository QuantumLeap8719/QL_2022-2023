package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionConstants {
    public static double lowerH = 160;
    public static double lowerS = 110;
    public static double lowerV = 80;
    public static double upperH = 180;
    public static double upperS = 255;
    public static double upperV = 255;

    public static double lowerHBlue = 0;
    public static double lowerSBlue = 140;
    public static double lowerVBlue = 70;
    public static double upperHBlue = 130;
    public static double upperSBlue = 255;
    public static double upperVBlue = 255;


    public static double STACK_HEIGHT = 9.6;

    public static double relX = 3.375;
    public static double relY = 3.95625;

    public static double dilationConstant = 10;
    public static double erosionConstant = 2.5;
    public static double blurConstant = 3;
    public static double LineFollowerTarget = 320;

    public static int minArea = 1000;
    public static int horizon = 230;
}

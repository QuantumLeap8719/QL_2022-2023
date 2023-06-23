package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Config
public class SleeveDetectorV2 extends OpenCvPipeline {
    public final Scalar COLOR = new Scalar(0, 255, 0);

    Mat HSVMat = new Mat();

    public static Rect BOUNDING_BOX  = new Rect(
            new Point(455, 250),
            new Point(440, 190)
    );;

    // Define the lower and upper thresholds for each color
    public static Scalar lowerPurple = new Scalar(130, 50, 50);
    public static Scalar upperPurple = new Scalar(170, 255, 255);
    public static Scalar lowerGreen = new Scalar(40, 100, 100);
    public static Scalar upperGreen = new Scalar(80, 255, 255);
    public static Scalar lowerYellow = new Scalar(20, 100, 100);
    public static Scalar upperYellow = new Scalar(40, 255, 255);

    public static double minArea = 1000;

    private double H = 0.0;

    private int coneCase = 0;

    public SleeveDetectorV2(){

    }

    @Override
    public Mat processFrame(Mat input){
        Mat cropped = new Mat(input, BOUNDING_BOX);

        Imgproc.rectangle(input, BOUNDING_BOX, COLOR);

        Imgproc.cvtColor(cropped, HSVMat, Imgproc.COLOR_BGR2HSV_FULL);

        // Create masks for each color
        Mat yellowMask = new Mat();
        Mat greenMask = new Mat();
        Mat purpleMask = new Mat();
        Core.inRange(HSVMat, lowerYellow, upperYellow, yellowMask);
        Core.inRange(HSVMat, lowerGreen, upperGreen, greenMask);
        Core.inRange(HSVMat, lowerPurple, upperPurple, purpleMask);

        // Find contours in the masks
        List<MatOfPoint> yellowContours = new ArrayList<>();
        List<MatOfPoint> greenContours = new ArrayList<>();
        List<MatOfPoint> purpleContours = new ArrayList<>();

        Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(greenMask, greenContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(purpleMask, purpleContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Determine the color of the object based on the largest contour area
        if (!yellowContours.isEmpty()) {
            double yellowArea = Imgproc.contourArea(yellowContours.get(0));
            if (yellowArea > minArea) {
                coneCase = 2;
            }
        } else if (!greenContours.isEmpty()) {
            double greenArea = Imgproc.contourArea(greenContours.get(0));
            if (greenArea > minArea) {
                coneCase = 1;
            }
        } else if (!purpleContours.isEmpty()) {
            double purpleArea = Imgproc.contourArea(purpleContours.get(0));
            if (purpleArea > minArea) {
                coneCase = 0;
            }
        }

        return yellowMask;
    }

    public int getCase(){
        return coneCase;
    }
}
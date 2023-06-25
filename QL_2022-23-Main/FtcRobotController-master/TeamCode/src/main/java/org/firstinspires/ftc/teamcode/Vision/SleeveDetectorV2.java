package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.BOUNDING_BOX;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerGreenH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerGreenS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerGreenV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerPurpleH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerPurpleS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerPurpleV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerYellowH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerYellowS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.lowerYellowV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperGreenH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperGreenS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperGreenV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperPurpleH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperPurpleS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperPurpleV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperYellowH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperYellowS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.upperYellowV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class SleeveDetectorV2 extends OpenCvPipeline {
    public final Scalar COLOR = new Scalar(0, 255, 0);

    Mat HSVMat = new Mat();

    public static double minArea = 1000;

    private double H = 0.0;

    private int coneCase = 0;

    @Override
    public Mat processFrame(Mat input){
        Scalar lowerPurple = new Scalar(lowerPurpleH, lowerPurpleS, lowerPurpleV);
        Scalar upperPurple = new Scalar(upperPurpleH, upperPurpleS, upperPurpleV);
        Scalar lowerGreen = new Scalar(lowerGreenH, lowerGreenS, lowerGreenV);
        Scalar upperGreen = new Scalar(upperGreenH, upperGreenS, upperGreenV);
        Scalar lowerYellow = new Scalar(lowerYellowH, lowerYellowS, lowerYellowV);
        Scalar upperYellow = new Scalar(upperYellowH, upperYellowS, upperYellowV);

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

        /*Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(greenMask, greenContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(purpleMask, purpleContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);*/

        Imgproc.GaussianBlur(purpleMask, purpleMask, new Size(VisionConstants.blurConstant, VisionConstants.blurConstant), 0);
        Imgproc.GaussianBlur(greenMask, greenMask, new Size(VisionConstants.blurConstant, VisionConstants.blurConstant), 0);
        Imgproc.GaussianBlur(yellowMask, yellowMask, new Size(VisionConstants.blurConstant, VisionConstants.blurConstant), 0);

        double[] means = new double[3];
        means[0] = Core.mean(purpleMask).val[0];
        means[1] = Core.mean(greenMask).val[0];
        means[2] = Core.mean(yellowMask).val[0];

        int maxIndex = 0;
        double largest = 0;

        for(int i = 0; i < 3; i++){
            if(means[i] > largest){
                maxIndex = i;
                largest = means[i];
            }
        }

        coneCase = maxIndex;

        //FtcDashboard.getInstance().getTelemetry().addData("Area", Imgproc.contourArea(purpleContours.get(0)));
        //FtcDashboard.getInstance().getTelemetry().update();

        return yellowMask;
    }

    public int getCase(){
        return coneCase;
    }
}

@Config
class SleeveDetectorV2Constants {
    public static double lowerPurpleH = 180;
    public static double lowerPurpleS = 40;
    public static double lowerPurpleV = 40;
    public static double upperPurpleH = 255;
    public static double upperPurpleS = 225;
    public static double upperPurpleV = 240;

    public static double lowerGreenH = 40;
    public static double lowerGreenS = 10;
    public static double lowerGreenV = 10;
    public static double upperGreenH = 130;
    public static double upperGreenS = 255;
    public static double upperGreenV = 255;

    public static double lowerYellowH = 120;
    public static double lowerYellowS = 70;
    public static double lowerYellowV = 40;
    public static double upperYellowH = 170;
    public static double upperYellowS = 255;
    public static double upperYellowV = 255;

    public static Rect BOUNDING_BOX  = new Rect(
            new Point(385, 280),
            new Point(310, 170)
    );;


}
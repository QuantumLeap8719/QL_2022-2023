package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.BOUNDING_BOX;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerGreenH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerGreenS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerGreenV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerPurpleH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerPurpleS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerPurpleV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerYellowH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerYellowS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.lowerYellowV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperGreenH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperGreenS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperGreenV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperPurpleH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperPurpleS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperPurpleV;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperYellowH;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperYellowS;
import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2ConstantsRight.upperYellowV;

import com.acmerobotics.dashboard.config.Config;

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
import java.util.List;


public class SleeveDetectorV2Right extends OpenCvPipeline {
    public final Scalar COLOR = new Scalar(0, 255, 0);

    Mat HSVMat = new Mat();

    public static double minArea = 1000;

    private double H = 0.0;

    private int coneCase = 0;

    public SleeveDetectorV2Right(){

    }

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

        Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(greenMask, greenContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(purpleMask, purpleContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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

        return purpleMask;
    }

    public int getCase(){
        return coneCase;
    }
}

@Config
class SleeveDetectorV2ConstantsRight {
    public static double lowerPurpleH = 180;
    public static double lowerPurpleS = 40;
    public static double lowerPurpleV = 40;
    public static double upperPurpleH = 255;
    public static double upperPurpleS = 225;
    public static double upperPurpleV = 240;

    public static double lowerGreenH = 90;
    public static double lowerGreenS = 20;
    public static double lowerGreenV = 120;
    public static double upperGreenH = 130;
    public static double upperGreenS = 240;
    public static double upperGreenV = 230;

    public static double lowerYellowH = 130;
    public static double lowerYellowS = 50;
    public static double lowerYellowV = 0;
    public static double upperYellowH = 140;
    public static double upperYellowS = 215;
    public static double upperYellowV = 255;

    public static Rect BOUNDING_BOX  = new Rect(
            new Point(595, 280),
            new Point(520, 170)
    );;


}
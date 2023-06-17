package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector3;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.opencv.calib3d.Calib3d;


public class LineFollower extends OpenCvPipeline {
    Mat output;
    Mat HSVMat;
    Telemetry telemetry;
    List<MatOfPoint> contoursList = new ArrayList<>(); //Array of all contours
    int numContoursFound = 0;
    private Scalar lowerHSV = new Scalar(VisionConstants.lowerH, VisionConstants.lowerS, VisionConstants.lowerV);
    private Scalar upperHSV = new Scalar(VisionConstants.upperH, VisionConstants.upperS, VisionConstants.upperV);
    public static Point midMaxPoint;

    public LineFollower(Telemetry telemetry){
        midMaxPoint = new Point(320, 160);
        output = new Mat();
        HSVMat = new Mat();
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input){
        contoursList.clear();
        Scalar lowerHSV = new Scalar(VisionConstants.lowerH, VisionConstants.lowerS, VisionConstants.lowerV);
        Scalar upperHSV = new Scalar(VisionConstants.upperH, VisionConstants.upperS, VisionConstants.upperV);

        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_BGR2HSV_FULL);
        Core.inRange(HSVMat, lowerHSV, upperHSV, output);

        Imgproc.GaussianBlur(output, output, new Size(VisionConstants.blurConstant, VisionConstants.blurConstant), 0);

        //Creating the kernal of 15, 15 structuring element
        Mat kernal = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(2 * VisionConstants.dilationConstant + 1, 2 * VisionConstants.dilationConstant + 1));
        Mat kernal2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(2 * VisionConstants.erosionConstant + 1, 2 * VisionConstants.erosionConstant + 1));
        //Dilating the image parameters are source, destination, and kernal
        Imgproc.erode(output, output, kernal2);
        Imgproc.dilate(output, output, kernal);

        //Fingding the contours based on the HSV ranges
        Imgproc.findContours(output, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if(contoursList.size() == 0){
            return input;
        }

        // Find the contour with the largest area
        double maxArea = 0;
        MatOfPoint maxContour = null;

        for (MatOfPoint contour : contoursList) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxContour = contour;
            }
        }

        MatOfPoint2f maxContour2f = new MatOfPoint2f();
        if(maxContour != null) {
            maxContour2f.fromArray(maxContour.toArray());
        }
        MatOfPoint2f contourPoly = new MatOfPoint2f();
        Imgproc.approxPolyDP(maxContour2f, contourPoly, 0.02 * Imgproc.arcLength(maxContour2f, true), true);
/////////////////
        List<Point> points = contourPoly.toList();

        Point maxPoint = null;
        Point secondMaxPoint = null;

        for (Point point : points) {
            if (maxPoint == null || point.y > maxPoint.y) {
                secondMaxPoint = maxPoint;
                maxPoint = point;
            } else if (secondMaxPoint == null || point.y > secondMaxPoint.y) {
                secondMaxPoint = point;
            }
        }

        System.out.println("Maximum point: " + maxPoint);
        System.out.println("Second maximum point: " + secondMaxPoint);

        // Draw the contour with the largest area on the image
        Imgproc.drawContours(input, Collections.singletonList(maxContour), -1, new Scalar(255), 2);

        if(maxPoint != null && secondMaxPoint != null) {
            Imgproc.drawMarker(input, maxPoint, new Scalar(0, 0, 255), 1, 5, 5);
            Imgproc.drawMarker(input, secondMaxPoint, new Scalar(0, 0, 255), 1, 5, 5);
            midMaxPoint = new Point((maxPoint.x + secondMaxPoint.x) / 2, (maxPoint.y + secondMaxPoint.y) / 2);
        }

        return input;
    }

}

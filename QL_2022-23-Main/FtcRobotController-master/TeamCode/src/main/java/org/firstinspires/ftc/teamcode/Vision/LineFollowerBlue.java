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
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.opencv.calib3d.Calib3d;


public class LineFollowerBlue extends OpenCvPipeline {
    Mat output;
    Mat HSVMat;
    Telemetry telemetry;
    List<MatOfPoint> contoursList = new ArrayList<>(); //Array of all contours
    int numContoursFound = 0;
    private Scalar lowerHSV = new Scalar(VisionConstants.lowerHBlue, VisionConstants.lowerSBlue, VisionConstants.lowerVBlue);
    private Scalar upperHSV = new Scalar(VisionConstants.upperHBlue, VisionConstants.upperSBlue, VisionConstants.upperVBlue);
    private ArrayList<Point> midMaxPoints;
    public static Point midMaxPoint;

    private static boolean empty;

    public LineFollowerBlue(Telemetry telemetry){
        midMaxPoint = new Point(320, 160);
        empty = false;
        midMaxPoints = new ArrayList<>();
        output = new Mat();
        HSVMat = new Mat();
        this.telemetry = telemetry;
    }

    public static boolean isEmpty(){
        return empty;
    }

    @Override
    public Mat processFrame(Mat input){
        midMaxPoints.clear();
        contoursList.clear();
        Scalar lowerHSV = new Scalar(VisionConstants.lowerHBlue, VisionConstants.lowerSBlue, VisionConstants.lowerVBlue);
        Scalar upperHSV = new Scalar(VisionConstants.upperHBlue, VisionConstants.upperSBlue, VisionConstants.upperVBlue);

        input = new Mat(input, new Rect(0, VisionConstants.horizon, 640, 320 - VisionConstants.horizon));
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
            empty = true;
            return input;
        }else{
            empty = false;
        }

        for (MatOfPoint contour : contoursList) {
            double area = Imgproc.contourArea(contour);
            if (area < VisionConstants.minArea) {
                continue;
            }

            MatOfPoint2f maxContour2f = new MatOfPoint2f();
            maxContour2f.fromArray(contour.toArray());
            MatOfPoint2f contourPoly = new MatOfPoint2f();
            Imgproc.approxPolyDP(maxContour2f, contourPoly, 0.02 * Imgproc.arcLength(maxContour2f, true), true);

            List<Point> points = contourPoly.toList();

            Point maxPoint = null;
            Point secondMaxPoint = null;

            for (Point point : points) {
                if (maxPoint == null || point.y < maxPoint.y) {
                    secondMaxPoint = maxPoint;
                    maxPoint = point;
                } else if (secondMaxPoint == null || point.y < secondMaxPoint.y) {
                    secondMaxPoint = point;
                }
            }

            System.out.println("Maximum point: " + maxPoint);
            System.out.println("Second maximum point: " + secondMaxPoint);

            // Draw the contour with the largest area on the image
            Imgproc.drawContours(input, Collections.singletonList(contour), -1, new Scalar(255), 2);

            if(maxPoint != null && secondMaxPoint != null) {
                Imgproc.drawMarker(input, maxPoint, new Scalar(0, 0, 255), 1, 5, 5);
                Imgproc.drawMarker(input, secondMaxPoint, new Scalar(0, 0, 255), 1, 5, 5);
                Point buff = new Point((maxPoint.x + secondMaxPoint.x) / 2, (maxPoint.y + secondMaxPoint.y) / 2);
                midMaxPoints.add(buff);
                Imgproc.putText(input, buff.toString(), buff, 1, 1.5, new Scalar(0, 255, 0));
            }
        }

        midMaxPoint = getMidMaxPoint();

        return input;
    }

    private Point getMidMaxPoint(){
        if(midMaxPoints.size() == 0){
            return new Point(320, 160);
        }else{
            Point closest_point = new Point();
            double closest_distance = 330;
            for(Point midMaxPoint : midMaxPoints){
                double distance = Math.abs(midMaxPoint.x - VisionConstants.LineFollowerTarget);
                if(distance < closest_distance) {
                    closest_distance = distance;
                    closest_point = midMaxPoint;
                }
            }
            return closest_point;
        }
    }
}

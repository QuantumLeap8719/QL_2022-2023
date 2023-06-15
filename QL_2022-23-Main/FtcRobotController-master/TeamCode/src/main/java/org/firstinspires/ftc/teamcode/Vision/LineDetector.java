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


public class LineDetector extends OpenCvPipeline {
    Mat output;
    Mat HSVMat;
    Telemetry telemetry;
    List<MatOfPoint> contoursList = new ArrayList<>(); //Array of all contours
    int numContoursFound = 0;
    private Scalar lowerHSV = new Scalar(VisionConstants.lowerH, VisionConstants.lowerS, VisionConstants.lowerV);
    private Scalar upperHSV = new Scalar(VisionConstants.upperH, VisionConstants.upperS, VisionConstants.upperV);
    public CalibrationParameters CALIB_PARAMS;

    public Vector3 adjustedPointMin;
    public Vector3 adjustedPointMax;
    public double angle = 0;


    public Point3 relPoint = new Point3(-3.375, -3.95625, 0);

    private Mat rotation;
    private Mat uvPointMax; //Pixel point on camera view
    private Mat uvPointMin; //Pixel point on camera view
    private Mat lhsMax;
    private Mat lhsMin;
    private Mat rhsMin;
    private Mat rhsMax;
    private Mat pointMatMin;
    private Mat pointMatMax;

    //Extrinsic Calibration Parameters
    public Mat RVEC; // Rotation Vector Of Camera found through Calibration
    public Mat TVEC; // Translation Vector Of Camera from origin found through Calibration
    private final Mat EMPTY_MAT;

    public LineDetector(Telemetry telemetry){
        output = new Mat();
        HSVMat = new Mat();
        this.telemetry = telemetry;

        adjustedPointMin = new Vector3();
        adjustedPointMax = new Vector3();

        EMPTY_MAT = new Mat();

        RVEC = new MatOfDouble(1.314441617728361, 1.295123922707693, -1.107900677635943);
        TVEC = new MatOfDouble(-5.107222576924736, 3.770677893266901, 16.66868459603753);
        CALIB_PARAMS = new CalibrationParameters(470.079, 467.252,
                353.972, 187.813, 0.0623386, -0.286305, -0.000192390, 0.00587140, 0.315253);

        rotation = new Mat(3, 3, CvType.CV_64FC1);
        uvPointMin = new Mat(3, 1, CvType.CV_64FC1);
        uvPointMax = new Mat(3, 1, CvType.CV_64FC1);
        lhsMin = new Mat();
        rhsMin = new Mat();
        lhsMax = new Mat();
        rhsMax = new Mat();
        pointMatMin = new Mat();
        pointMatMax = new Mat();
    }

    @Override
    public Mat processFrame(Mat input){
        relPoint = new Point3(-VisionConstants.relX, -VisionConstants.relY, 0);
        contoursList.clear();
        Scalar lowerHSV = new Scalar(VisionConstants.lowerH, VisionConstants.lowerS, VisionConstants.lowerV);
        Scalar upperHSV = new Scalar(VisionConstants.upperH, VisionConstants.upperS, VisionConstants.upperV);

        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_BGR2HSV_FULL);
        Core.inRange(HSVMat, lowerHSV, upperHSV, output);

        //Creating the kernal of 15, 15 structuring element
        Mat kernal = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(2 * VisionConstants.dilationConstant + 1, 2 * VisionConstants.dilationConstant + 1));
        Mat kernal2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(2 * VisionConstants.erosionConstant + 1, 2 * VisionConstants.erosionConstant + 1));
        //Dilating the image parameters are source, destination, and kernal
        Imgproc.erode(output, output, kernal2);
        Imgproc.dilate(output, output, kernal);

        //Fingding the contours based on the HSV ranges
        Imgproc.findContours(output, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
 /////////////
        Point minPoint = null;
        Point secondMinPoint = null;

        for (Point point : points) {
            if (minPoint == null || point.y < minPoint.y) {
                secondMinPoint = minPoint;
                minPoint = point;
            } else if (secondMinPoint == null || point.y < secondMinPoint.y) {
                secondMinPoint = point;
            }
        }

        System.out.println("Minimum point: " + minPoint);
        System.out.println("Second minimum point: " + secondMinPoint);
//////////////

        // Draw the contour with the largest area on the image
        Imgproc.drawContours(input, Collections.singletonList(maxContour), -1, new Scalar(255), 2);

        if(minPoint != null && secondMinPoint != null && maxPoint != null && secondMaxPoint != null) {
            Imgproc.drawMarker(input, minPoint, new Scalar(0, 0, 255), 1, 5, 5);
            Imgproc.drawMarker(input, secondMinPoint, new Scalar(0, 0, 255), 1, 5, 5);
            Point midMinPoint = new Point((minPoint.x + secondMinPoint.x)/2, (minPoint.y + secondMinPoint.y)/2);
            Imgproc.drawMarker(input, maxPoint, new Scalar(0, 0, 255), 1, 5, 5);
            Imgproc.drawMarker(input, secondMaxPoint, new Scalar(0, 0, 255), 1, 5, 5);
            Point midMaxPoint = new Point((maxPoint.x + secondMaxPoint.x)/2, (maxPoint.y + secondMaxPoint.y)/2);

            // undistort centroid based on camera parameters and reproject with identity matrix
            Mat cameraMatrix = CALIB_PARAMS.getCameraMatrix();
            MatOfDouble distCoeffs = CALIB_PARAMS.getDistCoeffs();

            MatOfPoint2f corrMinPoints = new MatOfPoint2f();
            Calib3d.undistortPoints(new MatOfPoint2f(midMinPoint), corrMinPoints, cameraMatrix, distCoeffs);
            Point corrMinPoint = corrMinPoints.toArray()[0];

            MatOfPoint2f corrMaxPoints = new MatOfPoint2f();
            Calib3d.undistortPoints(new MatOfPoint2f(midMaxPoint), corrMaxPoints, cameraMatrix, distCoeffs);
            Point corrMaxPoint = corrMaxPoints.toArray()[0];

            // rotation matrix from rotation vector
            Calib3d.Rodrigues(RVEC, rotation);

            // this based on the projection math in https://stackoverflow.com/questions/12299870 (the question)
            //Essentially reverse engineering the formula to calculate the world -> camera to go from camera -> world
            // by producing a line(similar triangles) to account for the reproduction of the loss of z axis
            uvPointMax.put(0, 0, corrMaxPoint.x, corrMaxPoint.y, 1);
            uvPointMin.put(0, 0, corrMinPoint.x, corrMinPoint.y, 1);

            // camera matrix not involved as undistortPoints reprojects with identity camera matrix
            Core.gemm(rotation.inv(), uvPointMax, 1, EMPTY_MAT, 0, lhsMax);
            Core.gemm(rotation.inv(), uvPointMin, 1, EMPTY_MAT, 0, lhsMin);
            Core.gemm(rotation.inv(), TVEC, 1, EMPTY_MAT, 0, rhsMin);
            Core.gemm(rotation.inv(), TVEC, 1, EMPTY_MAT, 0, rhsMax);

            double s1 = (VisionConstants.STACK_HEIGHT + rhsMin.get(2, 0)[0]) / lhsMin.get(2, 0)[0];
            double s2 = (rhsMax.get(2, 0)[0]) / lhsMax.get(2, 0)[0];
            Core.scaleAdd(uvPointMin, -s1, TVEC, pointMatMin);
            Core.scaleAdd(uvPointMax, -s2, TVEC, pointMatMax);
            Core.gemm(rotation.inv(), pointMatMin, -1, EMPTY_MAT, 0, pointMatMin);
            Core.gemm(rotation.inv(), pointMatMax, -1, EMPTY_MAT, 0, pointMatMax);

            double[] buffMin = new double[3];
            double[] buffMax = new double[3];
            pointMatMin.get(0, 0, buffMin);
            pointMatMax.get(0, 0, buffMax);
            Point3 pointMin = new Point3(buffMin);
            Point3 pointMax = new Point3(buffMax);
            adjustedPointMin = new Vector3(pointMin.y + relPoint.y, pointMin.x + relPoint.x, pointMin.z + relPoint.z);
            adjustedPointMax = new Vector3(pointMax.y + relPoint.y, pointMax.x + relPoint.x, pointMax.z + relPoint.z);

            angle = Math.atan2(adjustedPointMin.x - adjustedPointMax.x, adjustedPointMin.y - adjustedPointMax.y);
            adjustedPointMin.rotateZ(angle);
            adjustedPointMax.rotateZ(angle);
            telemetry.addData("AdjustedPointMin", adjustedPointMin);
            telemetry.addData("AdjustedPointMax", adjustedPointMax);
        }

        return input;
    }
}
package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DuckDetector extends OpenCvPipeline {
    Mat HSVmat = new Mat();
    Mat contoursOnFrameMat = new Mat();

    List<MatOfPoint> contoursList = new ArrayList<>();

    int numContoursFound = 0;

    public Scalar lowerHSV = new Scalar(46.8, 66.6, 80.8);
    public Scalar upperHSV = new Scalar(102.0, 255, 255);

    public double threshold = 500;
    public double horizon = 100;

    public double blurConstant = 1;

    public double dilationConstant = 2;

    int duckPosition = 0;

    Telemetry telemetryOpenCV = null;

    public DuckDetector(Telemetry OpModeTelemetry){
        telemetryOpenCV = OpModeTelemetry;
    }

    public int getDuckPosition(){
        return duckPosition;
    }

    @Override
    public Mat processFrame(Mat input) {
        //clear
        contoursList.clear();
        Imgproc.cvtColor(input, HSVmat, Imgproc.COLOR_RGB2HSV_FULL);

        //filters out all the colors but the yellow
        Core.inRange(HSVmat, lowerHSV, upperHSV, HSVmat);

        Size kernelSize = new Size(blurConstant, blurConstant);

        //blurs the image
        Imgproc.GaussianBlur(HSVmat, HSVmat, kernelSize, 0);

        Size kernelSize2 =  new  Size(2 * dilationConstant + 1, 2 * dilationConstant + 1);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize2);

        Imgproc.dilate(HSVmat, HSVmat, kernel);

        //finding the contours
        Imgproc.findContours(HSVmat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);

        for(MatOfPoint contour : contoursList){
            Rect rect = Imgproc.boundingRect(contour);

            if(rect.area() >= threshold && rect.y >= horizon){
                Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                Imgproc.putText(contoursOnFrameMat, String.valueOf(rect.x), rect.tl(), 0, 0.5, new Scalar(255, 255, 255));

                //Blue Side
                /*if(rect.x < 70){
                    duckPosition = 0;
                }else if(rect.x <= 160){
                    duckPosition = 1;
                }else{
                    duckPosition = 2;
                }*/

                //Red Side
                if(rect.x > 400){
                    duckPosition = 2;
                }else if(rect.x >= 300){
                    duckPosition = 1;
                }else{
                    duckPosition = 0;
                }
            }
        }

        return contoursOnFrameMat;
    }
}

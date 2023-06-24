package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


@Config
public class SleeveDetector extends OpenCvPipeline {
    public final Scalar BLUE = new Scalar(0, 0, 255);

    Mat matrix = new Mat();

    public static Rect BOUNDING_BOX  = new Rect(
            new Point(375, 250),
            new Point(360, 190)
    );;

    private double H = 0.0;

    public SleeveDetector(){

    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, matrix, Imgproc.COLOR_BGR2HSV_FULL);
        /*Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(matrix, lowHSV, highHSV, matrix);
        */
        Mat sleeveMatrix = matrix.submat(BOUNDING_BOX);

        H = (int)Core.sumElems(sleeveMatrix).val[0] / BOUNDING_BOX.area();


        Imgproc.rectangle(
                matrix, // Buffer to draw on
                BOUNDING_BOX,
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        FtcDashboard.getInstance().getTelemetry().addData("H", H);
        FtcDashboard.getInstance().getTelemetry().update();
        sleeveMatrix.release();
        return matrix;
    }

    public int getCase(){
        if(H < 110){
            return 2; //Blue = 10
        }else if(H < 200 && H > 110){
            return 1; //Green = 90
        }else if(H > 200){
            return 0; //Red = 160
        }
        return 0;
    }
}
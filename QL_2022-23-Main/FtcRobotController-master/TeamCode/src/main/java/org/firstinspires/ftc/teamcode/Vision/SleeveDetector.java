package org.firstinspires.ftc.teamcode.Vision;

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

    private double upperRingMatrix;
    private Telemetry telemetry;
    Mat matrix = new Mat();
    static double PERCENT_COLOR_THRESHOLD = 0.4;


    public static Rect BOUNDING_BOX = new Rect(
            new Point(230, 220),
            new Point(190, 150)
    );

    private double avg = 0.0;
    public SleeveDetector(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, matrix, Imgproc.COLOR_RGB2HSV);
        /*Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(matrix, lowHSV, highHSV, matrix);
        */
        Mat sleeveMatrix = matrix.submat(BOUNDING_BOX);

        avg = (int)Core.sumElems(sleeveMatrix).val[0];


        Imgproc.rectangle(
                matrix, // Buffer to draw on
                BOUNDING_BOX,
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        telemetry.addData("Sleeve raw value: ", (int)Core.sumElems(sleeveMatrix).val[0]);
        telemetry.addData("Case: ", getCase());
        telemetry.addData("avg: ", avg);

        Imgproc.putText(matrix, String.valueOf(avg) , BOUNDING_BOX.tl(), 0, 0.5, new Scalar(255, 255, 255));


        sleeveMatrix.release();

        telemetry.update();
        return matrix;
    }

    public int getCase(){
        if(avg < 180000){
            return 2; //yellow
        }else if(avg >= 180000 && avg <= 260000){
            return 0; //green = 250000
        }else if(avg > 260000){
            return 1; //blue = 330000
        }
        return 0;
    }
}
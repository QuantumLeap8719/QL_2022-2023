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
public class LineDetector extends OpenCvPipeline {
    Mat output;
    Telemetry telemetry;
    private Scalar lowerHSV = new Scalar(VisionConstants.lowerH, VisionConstants.lowerS, VisionConstants.lowerV);
    private Scalar upperHSV = new Scalar(VisionConstants.upperH, VisionConstants.upperS, VisionConstants.upperV);


    public LineDetector(Telemetry telemetry){
        output = new Mat();
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input){
        Core.inRange(input, lowerHSV, upperHSV, output);
        return output;
    }

}
@Config
class VisionConstants{
    public static double lowerH = 15;
    public static double lowerS = 90;
    public static double lowerV = 100;
    public static double upperH = 30;
    public static double upperS = 255;
    public static double upperV = 255;
}
package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class LineDetector extends OpenCvPipeline {
    Mat output;
    Mat HSVMat;
    Telemetry telemetry;
    private Scalar lowerHSV = new Scalar(VisionConstants.lowerH, VisionConstants.lowerS, VisionConstants.lowerV);
    private Scalar upperHSV = new Scalar(VisionConstants.upperH, VisionConstants.upperS, VisionConstants.upperV);

    public LineDetector(Telemetry telemetry){
        output = new Mat();
        HSVMat = new Mat();
        this.telemetry = telemetry;

    }

    @Override
    public Mat processFrame(Mat input){
        Scalar lowerHSV = new Scalar(VisionConstants.lowerH, VisionConstants.lowerS, VisionConstants.lowerV);
        Scalar upperHSV = new Scalar(VisionConstants.upperH, VisionConstants.upperS, VisionConstants.upperV);

        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(HSVMat, lowerHSV, upperHSV, output);

        return output;
    }

}
@Config
class VisionConstants{
    public static double lowerH = 156;
    public static double lowerS = 177;
    public static double lowerV = 64;
    public static double upperH = 210;
    public static double upperS = 241;
    public static double upperV = 255;
}
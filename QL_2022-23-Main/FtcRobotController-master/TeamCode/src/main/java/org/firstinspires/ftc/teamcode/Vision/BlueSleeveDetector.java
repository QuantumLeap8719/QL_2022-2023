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
public class BlueSleeveDetector extends OpenCvPipeline {
   public final Scalar BLUE = new Scalar(0, 0, 255);
   public static boolean blue = false;

   private double upperRingMatrix;
   private Telemetry telemetry;
   Mat matrix = new Mat();
   static double PERCENT_COLOR_THRESHOLD = 0.4;


   public static Rect BOUNDING_BOX = new Rect(
                 new Point(465, 135),
                 new Point(435, 80)
         );;

   private double avg = 0.0;

   private double S = 0.0;
   private double V = 0.0;
   public BlueSleeveDetector(Telemetry telemetry){
      this.telemetry = telemetry;
   }

   @Override
   public Mat processFrame(Mat input){
      Imgproc.cvtColor(input, matrix, Imgproc.COLOR_RGB2YCrCb);
        /*Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(matrix, lowHSV, highHSV, matrix);
        */
      Mat sleeveMatrix = matrix.submat(BOUNDING_BOX);

      avg = (int)Core.sumElems(sleeveMatrix).val[0];
      S = (int)Core.sumElems(sleeveMatrix).val[1];
      V = (int)Core.sumElems(sleeveMatrix).val[2];



      Imgproc.rectangle(
              matrix, // Buffer to draw on
              BOUNDING_BOX,
              BLUE, // The color the rectangle is drawn in
              2); // Thickness of the rectangle lines

      telemetry.addData("Sleeve raw value: ", (int)Core.sumElems(sleeveMatrix).val[0]);
      telemetry.addData("Case: ", getCase());
      telemetry.addData("avg: ", avg);
      telemetry.addData("S", S);
      telemetry.addData("V",V);

      Imgproc.putText(matrix, String.valueOf(S) , BOUNDING_BOX.tl(), 0, 0.5, new Scalar(255, 255, 255));


      sleeveMatrix.release();

      telemetry.update();
      return matrix;
   }

   public int getCase() {
      if(avg < 110000 ){
         return 1; //black = 72000
      }else if(avg >= 110000 && avg < 200000){
         return 2; //blue = 160000
      }else if(avg > 250000){
         return 0; //white = 400000
      }
      return 0;
   }
}
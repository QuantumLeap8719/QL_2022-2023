package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Vision.LineDetector;
import org.firstinspires.ftc.teamcode.Vision.SleeveDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class LineTester extends OpMode {
    OpenCvCamera webcam;
    LineDetector detector;
    Robot robot;
    Pose2d target;
    boolean first;

    @Override
    public void init() {
        first = true;
        target = new Pose2d();
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detector = new LineDetector(telemetry);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }


    @Override
    public void loop() {
        if(Math.abs(robot.getPos().getY()) < 15){
            target = new Pose2d(detector.adjustedPointMin.x + robot.getPos().getX(), -43, detector.angle);
        }
        robot.GoTo(target, new Pose2d(0.75, 0.75, 0.75));

        telemetry.addData("position", robot.getPos());
        telemetry.addData("target", target);
        robot.update();
    }
}

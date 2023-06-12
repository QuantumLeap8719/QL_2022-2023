package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Vision.LineDetector;
import org.firstinspires.ftc.teamcode.Vision.LineFollower;
import org.firstinspires.ftc.teamcode.Vision.SleeveDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class RunFollower extends OpMode {
    OpenCvCamera webcam;
    LineFollower detector;
    Robot robot;
    Pose2d target;
    boolean first;
    NormalizedColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        first = true;
        target = new Pose2d();
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detector = new LineFollower(telemetry);
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
    public void init_loop(){
        telemetry.addData("mid x pos", detector.midMaxPoint.x);
    }


    @Override
    public void loop() {
        robot.arm.V4BFrontHoldPos();
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH);
        if(Math.abs(robot.getPos().getY()) > 39){
            robot.drive.followLine(robot.getPos().getHeading(), 1, VisionConstants.LineFollowerTarget, distance, detector.midMaxPoint.x, 0.3, 0.3);
        }else if(Math.abs(robot.getPos().getY()) > 25){
            robot.drive.followLine(robot.getPos().getHeading(), -42, VisionConstants.LineFollowerTarget, robot.getPos().getY(), detector.midMaxPoint.x, 0.3, 0.3);
        }else{
            robot.drive.followLine(robot.getPos().getHeading(), -42, VisionConstants.LineFollowerTarget, robot.getPos().getY(), detector.midMaxPoint.x, 0.75, 0.75);
        }

        telemetry.addData("Distance", distance);
        telemetry.addData("position", robot.getPos());
        telemetry.addData("target", target);
        robot.update();
        robot.updatePos();
        robot.drive.write();
        robot.arm.write();
    }
}

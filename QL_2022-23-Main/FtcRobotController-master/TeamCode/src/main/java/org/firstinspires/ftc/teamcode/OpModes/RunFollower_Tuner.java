package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Vision.LineFollower;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RunFollower_Tuner extends OpMode {
    Robot robot;
    TelemetryPacket packet;
    OpenCvCamera webcam;
    LineFollower detector;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        packet = new TelemetryPacket();
        robot.setStartPose(new Pose2d(0, 0, 0));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        robot.arm.V4BHoldPos();
        robot.stopAndResetEncoders();

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
    public void loop() {
        robot.arm.V4BHoldPos();
        robot.drive.followLine(false, 0, VisionConstants.LineFollowerTarget, robot.getPos().getY(), LineFollower.midMaxPoint.x, 0.75, 0.75);
        robot.localizer.setPacket(packet);
        robot.updatePos();
        robot.drive.write();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

@Config
class RunFollower_TunerConstants {
    public static double x = 0;
    public static double y = 0;
    public static double theta = 0;
}
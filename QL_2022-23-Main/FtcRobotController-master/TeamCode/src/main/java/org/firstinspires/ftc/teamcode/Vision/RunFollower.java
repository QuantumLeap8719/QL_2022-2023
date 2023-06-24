package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.State;
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
    double bufferHeading;
    NormalizedColorSensor colorSensor;

    private enum STATE{
        DRIVING,
        GRABBING
    }

    STATE mRobotState = STATE.DRIVING;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        bufferHeading = 0;
        target = new Pose2d();
        robot = new Robot(hardwareMap, telemetry);
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
    public void init_loop(){
        telemetry.addData("mid x pos", LineFollower.midMaxPoint);
    }

    @Override
    public void loop() {
        robot.arm.manualSetPosition(0.11);

        robot.slides.setPosition(90);
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH);

        switch (mRobotState){

            case DRIVING:
                robot.arm.GrabberOpen();
                telemetry.addData("IS EMPTY?", LineFollower.isEmpty());

                double heading = 0;

                if(robot.getPos().getHeading() <= Math.PI){
                    heading = robot.getPos().getHeading();
                }else{
                    heading = -((2 * Math.PI ) - robot.getPos().getHeading());
                }

                if((Math.abs(Math.toDegrees(heading)) > 12 && Math.abs(robot.getPos().getY()) < 39) || LineFollower.isEmpty()) {
                    telemetry.addData("", "Camera failure... using odo.");
                    robot.GoTo(new Pose2d(0, -40, 0), new Pose2d(0.75, 0.75, 0.75));
                }else{
                    if (Math.abs(robot.getPos().getY()) > 35) {
                        robot.drive.followLine(true, 1.2, bufferHeading , distance, robot.getPos().getHeading(), 0.3, 0.3);
                        if(Math.abs(bufferHeading - robot.getPos().getHeading()) < Math.toRadians(1.5) && Math.abs(1.2 - distance) < 0.5){
                            mRobotState = STATE.GRABBING;
                        }
                    } else if (Math.abs(robot.getPos().getY()) > 25) {
                        bufferHeading = robot.getPos().getHeading();
                        robot.drive.followLine(false, -42, VisionConstants.LineFollowerTarget, robot.getPos().getY(), LineFollower.midMaxPoint.x, 0.3, 0.3);
                    } else {
                        bufferHeading = robot.getPos().getHeading();
                        robot.drive.followLine(false, -42, VisionConstants.LineFollowerTarget, robot.getPos().getY(), LineFollower.midMaxPoint.x, 0.75, 0.75);
                    }
                    robot.drive.write();
                    robot.updatePos();
                }
                break;
            case GRABBING:
                robot.drive.setPower(0,0,0);
                robot.arm.GrabberClose();
                robot.drive.write();
                break;
        }


        telemetry.addData("Distance", distance);
        telemetry.addData("position", robot.getPos());
        telemetry.addData("target", target);
        telemetry.addData("State", mRobotState);
        telemetry.addData("BufferHeading", Math.toDegrees(bufferHeading));
        robot.arm.write();

        robot.update();
    }
}

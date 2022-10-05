package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;
import org.firstinspires.ftc.teamcode.OpModes.LinearTeleOp;
import org.firstinspires.ftc.teamcode.Vision.DuckDetector;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class Robot {
    public Mecanum_Drive drive;
    public V4B_Arm arm;
    public Intake intake;
    public Carousel carousel;
    public Slides slides;
    public S4T_Localizer localizer;
    private S4T_Encoder encoderLY;
    private S4T_Encoder encoderLX;
    private S4T_Encoder encoderRY;
    private S4T_Encoder encoderRX;
    private HardwareMap hardwareMap;
    OpenCvCamera webcam;
    OpenCvPipeline detector;

    private Telemetry telemetry;

    public static Pose2d startPos = new Pose2d(0, 0, 0);
    List<LynxModule> allHubs;

    public Robot(HardwareMap map, Telemetry telemetry){
        this.hardwareMap = map;
        this.telemetry = telemetry;
        startPos = new Pose2d(0, 0, 0);


        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        encoderLY = new S4T_Encoder(map, "back_left");
        encoderLX = new S4T_Encoder(map, "front_left");
        encoderRY = new S4T_Encoder(map, "back_right");
        encoderRX = new S4T_Encoder(map, "front_right");

        carousel = new Carousel(hardwareMap, telemetry);

        drive = new Mecanum_Drive(map, telemetry);
        arm = new V4B_Arm(map);
        intake = new Intake(map, telemetry);
        slides = new Slides(map, telemetry);

        localizer = new S4T_Localizer(telemetry);

        telemetry.addData("Localizer Position", localizer.getPose());
        telemetry.update();
    }

    public void setBlue(){
        drive.setBlue();
        localizer.blue = true;
    }

    public void operate(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {
        /*if(slides.mRobotState == Slides.STATE.AUTOMATION){
            drive.driveCentric(gamepad1ex.gamepad, 0.5,1.0, 1.0, getPos().getHeading());
        }else{
            drive.driveCentric(gamepad1ex.gamepad, 1.0, 1.0, getPos().getHeading());
        }*/
        drive.driveCentric(gamepad1ex.gamepad, 1.0, 1.0, getPos().getHeading());

        //drive.drive(gamepad1ex.gamepad, 1.0, 1.0);

        intake.intake(gamepad1ex, gamepad2ex, telemetry);

        arm.operate(gamepad1ex, gamepad2ex, telemetry);
        carousel.operate(gamepad2ex.gamepad);

        if(gamepad1ex.isPress(GamepadEx.Control.start)){
            localizer.reset();
        }

        slides.operate(gamepad1ex, gamepad2ex);

        arm.write();
        intake.write();
        if(LinearTeleOp.robotState == LinearTeleOp.mRobotState.DRIVE){
            drive.write();
        }
        carousel.write();
        slides.write();

        telemetry.addData("Robot Position:", getPos());
        gamepad1ex.loop();
        gamepad2ex.loop();
        updatePos();
    }

    public void setStartPose(Pose2d startPos){
        this.startPos = startPos;
        localizer.setHeading(startPos.getHeading());
    }


    public void initializeWebcam(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detector = new DuckDetector(telemetry);
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
    public int getDuckCase(){
       return ((DuckDetector)detector).getDuckPosition();
    }

    public void stopWebcam(){
        webcam.stopStreaming();
    }

    public void updatePos(){
        encoderLX.update();
        encoderLY.update();
        encoderRX.update();
        encoderRY.update();
        localizer.update(getRawLeft_X_Dist(), getRawLeft_Y_Dist(), getRawRight_X_Dist(), getRawRight_Y_Dist());
    }

    public double getRawLeft_X_Dist(){
        return encoderLX.distance;
    }

    public double getRawRight_X_Dist(){
        return encoderRX.distance;
    }

    public double getRawLeft_Y_Dist(){
        return encoderLY.distance;
    }

    public double getRawRight_Y_Dist(){
        return encoderRY.distance;
    }

    public Pose2d getPos(){
        return new Pose2d(localizer.getPose().getX() + startPos.getX(), localizer.getPose().getY() + startPos.getY(), localizer.getPose().getHeading());
    }

    public Pose2d getStartPos(){
        return startPos;
    }

    public void GoTo(Pose2d pose, Pose2d speedLimits){
        updateGoTo(pose, speedLimits);
    }

    public void GoTo(double x, double y, double heading, double maxspeed_x, double maxspeed_y, double maxspeed_z){
        updateGoTo(new Pose2d(x, y, heading), new Pose2d(maxspeed_x, maxspeed_y, maxspeed_z));
    }

    public void setAngle(double heading){
        localizer.setHeading(heading);
    }

    private void updateGoTo(Pose2d pose, Pose2d speedLimits){
        drive.goToPoint(pose, getPos(), speedLimits.getX(), speedLimits.getY(), speedLimits.getHeading());
        telemetry.addData("Position: ", getPos());
        telemetry.addData("Target Position: ", pose);
        drive.write();
        updatePos();
    }
}

package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer_3;
import org.firstinspires.ftc.teamcode.OpModes.LinearTeleOp;
import org.firstinspires.ftc.teamcode.Vision.BlueSleeveDetector;
import org.firstinspires.ftc.teamcode.Vision.LineFollower;
import org.firstinspires.ftc.teamcode.Vision.LineFollowerBlue;
import org.firstinspires.ftc.teamcode.Vision.SleeveDetector;
import org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2;
import org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Right;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class Robot {
    public Mecanum_Drive drive;
    public V4B_Arm arm;
    public Slides slides;
    public S4T_Localizer_3 localizer;
    private S4T_Encoder encoderLY;
    //private S4T_Encoder encoderLX;
    private S4T_Encoder encoderRY;
    private S4T_Encoder encoderRX;
    private HardwareMap hardwareMap;
    private int cameraMonitorViewId;
    private boolean driveToggle = false;

    OpenCvCamera webcam;
    public OpenCvPipeline detector;
    OpenCvPipeline blueDetector;

    private Pose2d OFFSET = new Pose2d(0, 0, 0);

    private Telemetry telemetry;

    public static Pose2d startPos = new Pose2d(0, 0, 0);
    List<LynxModule> allHubs;

    public Robot(HardwareMap map, Telemetry telemetry){
        cameraMonitorViewId = -2;
        this.hardwareMap = map;
        this.telemetry = telemetry;
        startPos = new Pose2d(0, 0, 0);


        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        encoderLY = new S4T_Encoder(map, "fleft"); //port 0
        //encoderLX = new S4T_Encoder(map, "fright"); //port 1
        encoderRY = new S4T_Encoder(map, "bleft"); //port 2
        encoderRX = new S4T_Encoder(map, "bright"); //port 3


        drive = new Mecanum_Drive(map, telemetry);
        arm = new V4B_Arm(map);
        slides = new Slides(map, telemetry);

        localizer = new S4T_Localizer_3(telemetry);
        telemetry.addData("Localizer Position", localizer.getPose());
        telemetry.update();
    }

    public void operate(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {
        telemetry.addLine("MAKE SURE TO HIT RIGHT TRIGGER");
        //drive.setPower(0.5,0.5,0.5,0.5);



        arm.operate(gamepad1ex, gamepad2ex, telemetry);

        slides.operate(gamepad1ex, gamepad2ex);

        drive.write();
        arm.write();
        slides.write();

        telemetry.addData("Robot Position:", getPos());
        gamepad1ex.loop();
        gamepad2ex.loop();

        if(gamepad1ex.isPress(GamepadEx.Control.start)){
            telemetry.addLine("Resetting...");
            resetOdo();
        }else{
            updatePos();
            update();
        }

        if(gamepad2ex.isPress(GamepadEx.Control.start)){
            driveToggle = !driveToggle;
        }

        if(driveToggle){
            drive.drive(gamepad1ex.gamepad, 1.0, 0.8);
        } else {
            drive.driveCentric(gamepad1ex.gamepad, 1, 0.8, getPos().getHeading());
        }

        telemetry.addData("DriveToggle", driveToggle);
    }

    public void resetOdo(){
        encoderLY.reset();
        encoderRY.reset();
        encoderRX.reset();
        localizer.reset();
    }

    public void stopAndResetEncoders(){
        encoderLY.stopandreset();
        encoderRY.stopandreset();
        encoderRX.stopandreset();
        localizer.reset();
    }

    public void setStartPose(Pose2d startPos){
        this.startPos = startPos;
        localizer.setHeading(startPos.getHeading());
    }


    public void initializeWebcam(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        detector = new SleeveDetectorV2();
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

    public void blueInitializeWebcam(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        detector = new SleeveDetectorV2Right();
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

    public void closeCamera(){
        webcam.closeCameraDevice();
    }

    public void closeCameraAsync(){webcam.closeCameraDeviceAsync(()-> System.out.println("closed cam"));}

    public void coneWebcam(){
       /*
        if(cameraMonitorViewId == -2){
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        }

        */
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

    public void blueConeWebcam(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detector = new LineFollowerBlue(telemetry);
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


    public int getConeCase(){
       return ((SleeveDetectorV2)detector).getCase();
    }

    public int rightConeCase(){
        {
            return ((SleeveDetectorV2Right) detector).getCase();
        }
    }

    public void stopWebcam(){
        webcam.stopStreaming();
    }

    public void update(){
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    public void updatePos(){
        //encoderLX.update();
        encoderLY.update();
        encoderRX.update();
        encoderRY.update();
        localizer.update(getRawLeft_Y_Dist(),getRawRight_X_Dist(), getRawRight_Y_Dist());
    }

    /*public double getRawLeft_X_Dist(){
        return encoderLX.distance;
    }

     */

    public double getRawRight_X_Dist(){
        return encoderRX.distance;
    }

    public double getRawLeft_Y_Dist(){
        return encoderLY.distance;
    }

    public double getRawRight_Y_Dist(){
        return -encoderRY.distance;
    }

    public Pose2d getPos(){
        return new Pose2d((localizer.getPose().getX() + startPos.getX() - OFFSET.getX()), (localizer.getPose().getY() + startPos.getY() - OFFSET.getY()), localizer.getPose().getHeading());
    }

    public Pose2d getStartPos(){
        return startPos;
    }

    public void setOffset(Pose2d offset){
        this.OFFSET = offset;
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




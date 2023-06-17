package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Vision.*;

import java.util.ArrayList;

@Autonomous(name="Auto_NoCamera")
public class Auto_No_Camera extends LinearOpMode {

    private enum State {
        DRIVE_TO_DEPOSIT_PRELOAD,
        PRELOAD_DEPOSIT,
        PRELOAD_FORWARD,
        DRIVE_TO_INTAKE,
        GRAB,
        DRIVE_TO_DEPOSIT_MID,
        MID_FORWARD,
        DRIVE_TO_DEPOSIT_HIGH_FAR,
        HIGH_FAR_FORWARD,
        DEPOSIT,
        PARK,
        IDLE
    }

    State mRobotState = State.DRIVE_TO_DEPOSIT_PRELOAD;

    public Pose2d PRE_LOAD_CLEAR = new Pose2d(8, 23, Math.toRadians(0));
    public Pose2d PRE_LOAD_CLEAR2 = new Pose2d(2.5, 47.1, Math.toRadians(0));
    public Pose2d PRE_LOAD_DEPOSIT = new Pose2d(11.5, 63, Math.toRadians(36.5));
    public Pose2d PRE_LOAD_DEPOSIT_FORWARD = new Pose2d(2, 50, Math.toRadians(90));
    public Pose2d INTAKE_CLEAR = new Pose2d(0, 52, Math.toRadians(90));
    public Pose2d INTAKE_FAR_CLEAR = new Pose2d(13, 52, Math.toRadians(90));
    public Pose2d BACk_FAR_CLEAR = new Pose2d(13, 55, Math.toRadians(90));
    public Pose2d DEPOSIT_HIGH_FAR_CLEAR = new Pose2d(27, 54, Math.toRadians(90));
    public static Pose2d DEPOSIT_HIGH_FAR = new Pose2d(36.5, 44, Math.toRadians(121)); //SECOND HIGH


    public Pose2d DEPOSIT_HIGH = new Pose2d(1, 49, Math.toRadians(65));
    public Pose2d DEPOSIT_MID = new Pose2d(12, 42.4, Math.toRadians(116));
    public Pose2d DEPOSIT_MID_FORWARD = new Pose2d(4, 45, Math.toRadians(116));
    public static Pose2d DEPOSIT_HIGH_FAR_FORWARD = new Pose2d(31, 57, Math.toRadians(121)); //SECOND HIGH
    public Pose2d DEPOSIT_HIGH_FAR_FORWARD_CLEAR = new Pose2d(10, 54, Math.toRadians(90));


    //public Pose2d DEPOSIT_HIGH_FAR = new Pose2d(-19.5, -43.5, Math.toRadians(113));

    public Pose2d GRAB = new Pose2d(-19, 51.25, Math.toRadians(90));
    public Pose2d GRAB2 = new Pose2d(-19, 51.25, Math.toRadians(270));
    public Pose2d GRAB3 = new Pose2d(-19, 51.75, Math.toRadians(270));
    public Pose2d GRAB4 = new Pose2d(-19, 52.75, Math.toRadians(270));
    public Pose2d GRAB5 = new Pose2d(-19, 53, Math.toRadians(270));

    public static Pose2d PARK_CASE_1 = new Pose2d(24, -52.5, Math.toRadians(90));
    public static Pose2d PARK_CASE_3 = new Pose2d(-18.7, -52.5, Math.toRadians(90));
    public static Pose2d PARK_CASE_2 = new Pose2d(3.2, -52.5, Math.toRadians(90));

    double coneCase;
    boolean gtp = false;
    boolean isDown = true;
    int cycle = 0;

    //double armCycleOne = 0;
    //double armCycleTwo = 0.17;
    // double armCycleThree = 0.10;
    double armCycleFour = 0.11;
    double armCycleFive = 0.0;

    double slideHeightOne = 120;
    double slideHeightTwo = 105;
    double slideHeightThree = 70;
    double slideHeightFour = 45;

    int depositHeightPreload = 530;
    int depositHeightMid = 340;
    int depositHeightFarHigh = 550;

    double grabberCycleOne = 0.6;
    double grabberCycleTwo = 0.66;
    double grabberCycleThree = 0.64;
    double grabberCycleFour = V4B_Arm.grabberOpen;
    double grabberCycleFive = V4B_Arm.grabberClose;

    int numCycles = 5;
    ElapsedTime time;
    NormalizedColorSensor colorSensor;
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        time = new ElapsedTime();
        robot.localizer.reset();
        robot.setStartPose(new Pose2d(0, 0, 0));
        robot.arm.GrabberClose();
        robot.arm.V4BOutPose();
        robot.arm.write();

        boolean slidesKickout = false;

        //robot.initializeWebcam();
        //robot.coneWebcam();
        while (!isStarted() && !isStopRequested()) {
            //coneCase = robot.getConeCase();
            coneCase = 0;
            telemetry.addData("Case", coneCase);
            telemetry.update();
        }


        //robot.stopWebcam();

        waitForStart();

        time.startTime();


        while (opModeIsActive()) {
            ArrayList<CurvePoint> points = new ArrayList<>();

            switch (mRobotState) {
                case DRIVE_TO_DEPOSIT_PRELOAD:
                    points.add(new CurvePoint(new Pose2d(0, 0, 0),0.8,0.8,10));
                    points.add(new CurvePoint(PRE_LOAD_CLEAR,0.8,0.8,10));
                    points.add(new CurvePoint(PRE_LOAD_CLEAR2,0.6,0.6,10));
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,0.4,0.4,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.5 && Math.abs(robot.slides.getPosition() - depositHeightPreload) < 13 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1.5)) {
                        newState(State.PRELOAD_DEPOSIT);
                    }else{
                        time.reset();
                        robot.arm.V4BOutPose();
                        robot.arm.GrabberClose();
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 32.5){
                            robot.slides.setPosition(depositHeightPreload);
                        }
                    }
                    break;

                case PRELOAD_DEPOSIT:
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,1.0,1.0,10));
                    robot.slides.setPosition(depositHeightPreload - 50, -0.3, 1);

                    if(time.time() < 0.2) {
                        robot.arm.GrabberDeposit();
                    }

                    if(time.time() > 0.3 && time.time() < 0.4){
                        robot.arm.GrabberClose();
                    }

                    if(time.time() > 0.4){
                        robot.arm.V4BAutoHold();
                    }

                    if(time.time() > 0.6) {
                        newState(State.PRELOAD_FORWARD);
                    }
                    break;
                case PRELOAD_FORWARD:
                    robot.slides.setPosition(depositHeightPreload - 50, -0.3, 1);
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,1.0,1.0,10));
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT_FORWARD,1.0,1.0,10));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.0 && Math.abs(robot.getPos().getHeading() - Math.toRadians(90)) < Math.toRadians(5)) {
                        newState(State.DRIVE_TO_INTAKE);
                    } else {
                        time.reset();
                        robot.arm.V4BFrontPose();
                        robot.arm.GrabberOpen();
                        if (robot.slides.isDown()) {
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                            isDown = false;
                        } else {
                            robot.slides.setPower(-0.26);
                        }
                    }
                    break;

                case DRIVE_TO_INTAKE:

                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.26);
                    }
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT_FORWARD,1.0,1.0,10));

                    if(cycle >= 2){
                        points.add(new CurvePoint(DEPOSIT_HIGH_FAR_FORWARD_CLEAR, 1.0, 1.0, 10));
                    }else {
                        points.add(new CurvePoint(INTAKE_CLEAR, 1.0, 1.0, 10));
                    }

                    double speed = 1.0;

                    if (robot.getPos().vec().distTo(GRAB.vec()) < 15) {
                        speed = 0.5;
                    }

                    if(cycle == 0) {
                        points.add(new CurvePoint(GRAB, speed, speed, 10));
                    } else if(cycle == 1){
                        points.add(new CurvePoint(GRAB, speed, speed, 10));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(GRAB, speed, speed, 10));
                    } else if (cycle == 3){
                        points.add(new CurvePoint(GRAB, speed, speed, 10));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(GRAB, speed, speed, 10));
                    }else {
                        points.add(new CurvePoint(GRAB, speed, speed, 10));
                    }

                    if(cycle==0){
                        robot.slides.setPosition(slideHeightOne, -0.2501, 1);
                    }else if(cycle==1){
                        robot.slides.setPosition(slideHeightTwo, -0.2501, 1);
                    }else if(cycle==2){
                        robot.slides.setPosition(slideHeightThree, -0.2501, 1);
                    } else {
                        robot.slides.setPosition(slideHeightFour, -0.2501, 1);
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1.0)) {
                        newState(State.GRAB);
                    }else{
                        if(cycle == 0 || cycle == 1) {
                            robot.arm.manualSetPosition(armCycleFour);
                        }/* else if(cycle == 1){
                            robot.arm.manualSetPosition(armCycleTwo);
                            robot.arm.grabberPos(grabberCycleTwo);
                            //robot.slides.setPosition(85, -0.2501, 1);
                        } */else if(cycle == 2){
                            robot.arm.manualSetPosition(armCycleFour);
                        } else if (cycle == 3){
                            robot.arm.manualSetPosition(armCycleFour);
                        } else if (cycle == 4){
                            robot.arm.manualSetPosition(armCycleFive);
                        }

                        if(time.time() > 0.15) {
                            robot.arm.grabberPos(grabberCycleFour);
                        }
                    }
                    break;
                case GRAB:
                    points.add(new CurvePoint(GRAB,0.8,0.8,10));
                    if (cycle == 0) {
                        robot.slides.setPosition(slideHeightOne - 30, -0.3, 1);
                    } else if (cycle == 1) {
                        robot.slides.setPosition(slideHeightTwo - 30, -0.3, 1);
                    } else if (cycle == 2) {
                        robot.slides.setPosition(slideHeightThree - 30, -0.3, 1);
                    }else if (cycle == 3){
                        robot.slides.setPosition(slideHeightFour - 30, -0.3, 1);
                    }

                    if(time.time() > 0.1) {
                        robot.arm.GrabberClose();
                    }
                    if(time.time() > 0.4){
                        robot.arm.V4BOutPose();
                    }
                    if(time.time() > 0.9) {
                        if(cycle == 0) {
                            newState(State.DRIVE_TO_DEPOSIT_MID);
                        } else {
                            newState(State.DRIVE_TO_DEPOSIT_HIGH_FAR);
                        }
                    }
                    break;
                case DRIVE_TO_DEPOSIT_MID:
                    points.add(new CurvePoint(GRAB,0.8,0.8,10));
                    points.add(new CurvePoint(INTAKE_CLEAR,0.7,0.7,10));
                    points.add(new CurvePoint(DEPOSIT_MID,0.7,0.7,10));

                    robot.slides.setPosition(depositHeightMid);

                    robot.arm.GrabberClose();

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.25 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1)) {
                        newState(State.DEPOSIT);
                    }
                    break;

                case MID_FORWARD:
                    points.add(new CurvePoint(DEPOSIT_MID,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_MID_FORWARD,1.0,1.0,10));
                    points.add(new CurvePoint(INTAKE_CLEAR,1.0,1.0,10));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5) {
                        isDown = true;
                        newState(State.DRIVE_TO_INTAKE);
                    } else {
                        time.reset();
                        robot.arm.V4BAutoHold();
                        robot.slides.setPosition(depositHeightMid - 50, -0.3, 1);
                        if (robot.slides.isDown()) {
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                            isDown = false;
                        } else {
                            robot.slides.setPower(-0.26);
                        }
                    }
                    break;
                case DRIVE_TO_DEPOSIT_HIGH_FAR:
                    double slideHeight = 0;

                    points.add(new CurvePoint(GRAB,0.8,0.8,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_CLEAR,0.6,0.6,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR,0.4,0.4,10));

                    robot.arm.GrabberClose();

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 33) {
                        if(cycle == 2){
                            slideHeight = depositHeightFarHigh - 10;
                        } else {
                            slideHeight = depositHeightFarHigh;
                        }
                        robot.slides.setPosition(slideHeight);
                    }else{
                        robot.slides.setPower(0);
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.25) {
                        newState(State.DEPOSIT);
                    }
                    break;

                case HIGH_FAR_FORWARD:
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_FORWARD,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_FORWARD_CLEAR,1.0,1.0,10));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.0  && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(2.0)) {
                        isDown = true;
                        newState(State.DRIVE_TO_INTAKE);
                    } else {
                        time.reset();
                        robot.arm.V4BAutoHold();
                        robot.slides.setPosition(depositHeightFarHigh - 50, -0.3, 1);
                        if (robot.slides.isDown()) {
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                            isDown = false;
                        } else {
                            robot.slides.setPower(-0.26);
                        }
                    }
                    break;

                case DEPOSIT:
                    if(cycle == 0) {
                        points.add(new CurvePoint(DEPOSIT_MID, 1.0, 1.0, 10));
                    } else {
                        points.add(new CurvePoint(DEPOSIT_HIGH_FAR,1.0,1.0,10));
                    }

                    robot.slides.setPosition((cycle == 0 ? depositHeightMid : depositHeightFarHigh) - 50, -0.3, 1);

                    if (time.time() < 0.2) {
                        robot.arm.GrabberDeposit();
                    }

                    if (time.time() > 0.3 && time.time() < 0.4) {
                        robot.arm.GrabberClose();
                    }

                    if(time.time() > 0.4) {
                        if ((cycle + 1) == numCycles) {
                            newState(State.PARK);
                        } else {
                            cycle++;
                            if (cycle == 1) {
                                newState(State.MID_FORWARD);
                            } else {
                                newState(State.HIGH_FAR_FORWARD);
                            }
                        }
                    }

                    break;
                case PARK:
                    gtp = true;
                    if(coneCase == 0){
                        points.add(new CurvePoint(PARK_CASE_1,1.0,1.0,15));
                    }else if(coneCase == 1){
                        points.add(new CurvePoint(PARK_CASE_2,1.0,1.0,15));
                    }else if(coneCase == 2){
                        points.add(new CurvePoint(PARK_CASE_3,1.0,1.0,15));
                    }
                    robot.arm.V4BFrontHoldPos();
                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.26);
                    }
                    break;
                case IDLE:
                    robot.drive.setPower(0, 0, 0);
                    //robot.stopWebcam();
                    break;
            }

            if (points.size() != 0) {
                if (!gtp) {
                    RobotMovement.followCurve(points, robot, telemetry);
                } else {
                    robot.GoTo(points.get(points.size() - 1).toPose(), new Pose2d(points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).turnSpeed));
                }
            } else if(mRobotState == State.DRIVE_TO_INTAKE){
                robot.drive.write();
                robot.updatePos();
            }else {
                robot.drive.setPower(0, 0, 0,0);
                robot.drive.write();
                robot.updatePos();
            }


            robot.arm.write();
            robot.slides.write();
            robot.update();

            for(int i = 0; i < points.size(); i++){
                telemetry.addData("Point" + i, points.get(i));
            }

            telemetry.addData("State", mRobotState);
            telemetry.addData("Position", robot.getPos());
            telemetry.addData("Cycle", cycle);
            telemetry.addData("Grabber position", robot.arm.grabber.getPosition());
            telemetry.addData("Heading Error", Math.abs(robot.getPos().getHeading() - Math.toRadians(270)));
            telemetry.addData("IsDown", isDown);
            telemetry.addData("Touch Sensor", robot.slides.isDown());
            telemetry.update();
        }
    }

    public void updateSlidesDown(){
        if(robot.slides.isDown()){
            robot.slides.reset();
            robot.slides.setPower(0.0);
        } else {
            robot.slides.setPower(-0.2501);
        }
    }

    public void newState (State state){
        time.reset();
        mRobotState = state;
    }
}

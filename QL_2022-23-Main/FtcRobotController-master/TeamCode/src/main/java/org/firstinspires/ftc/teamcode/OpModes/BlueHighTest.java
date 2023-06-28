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
import org.firstinspires.ftc.teamcode.Vision.LineFollowerBlue;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;

import java.util.ArrayList;

@Autonomous(name="BlueHighTest")
public class BlueHighTest extends LinearOpMode {

    private enum State {
        PRELOAD_STRAFE,
        PRELOAD_MOVE,
        PRELOAD_DEPOSIT,
        PRELOAD_TURN,
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

    State mRobotState = State.PRELOAD_STRAFE;

    public Pose2d OFFSET = new Pose2d(0,0,0);
    
    public Pose2d PRE_LOAD_STRAFE = new Pose2d(17.5, 1, Math.toRadians(0));
    public Pose2d PRE_LOAD_CLEAR = new Pose2d(21, 22, Math.toRadians(90));
    public Pose2d PRE_LOAD_DEPOSIT = new Pose2d(29.5, 36.4, Math.toRadians(30));
    public Pose2d PRE_LOAD_DEPOSIT_TURN = new Pose2d(26.9, 42.5, Math.toRadians(110));
    public Pose2d PRE_LOAD_DEPOSIT_FORWARD = new Pose2d(-7, 54, Math.toRadians(110));
    public Pose2d PRE_LOAD_DEPOSIT_FORWARDGRAB = new Pose2d(-10, 52, Math.toRadians(90));
    public Pose2d INTAKE_CLEAR = new Pose2d(-5, 51, Math.toRadians(90));
    public Pose2d INTAKE_FAR_CLEAR = new Pose2d(13, 52, Math.toRadians(0));
    public Pose2d BACk_FAR_CLEAR = new Pose2d(13, 55, Math.toRadians(0));
    public Pose2d DEPOSIT_HIGH_FAR_CLEAR = new Pose2d(11.606, 51.937, Math.toRadians(90));
    public static Pose2d DEPOSIT_HIGH_FAR1 = new Pose2d(29.348, 43.399, Math.toRadians(134)); //SECOND HIGH
    public static Pose2d DEPOSIT_HIGH_FAR2 = new Pose2d(30, 42, Math.toRadians(134)); //SECOND HIGH
    public static Pose2d DEPOSIT_HIGH_FAR3 = new Pose2d(36.8, 43.4, Math.toRadians(129)); //SECOND HIGH
    public static Pose2d DEPOSIT_HIGH_FAR4 = new Pose2d(36.8, 43.4, Math.toRadians(129)); //SECOND HIGH


    public Pose2d DEPOSIT_HIGH = new Pose2d(1, 49, Math.toRadians(65));
    public Pose2d DEPOSIT_MID = new Pose2d(4.8, 42, Math.toRadians(116));
    public Pose2d DEPOSIT_MID_FORWARD = new Pose2d(-1, 50, Math.toRadians(90));
    public static Pose2d DEPOSIT_HIGH_FAR_FORWARD = new Pose2d(15, 57, Math.toRadians(90)); //SECOND HIGH
    public Pose2d DEPOSIT_HIGH_FAR_FORWARD_CLEAR = new Pose2d(2, 51.5, Math.toRadians(90));


    //public Pose2d DEPOSIT_HIGH_FAR = new Pose2d(-19.5, -43.5, Math.toRadians(113));

    public Pose2d OG_GRAB = new Pose2d(-20.406, 53.301, Math.toRadians(90));
    public Pose2d GRAB = new Pose2d(-28, 51.25, Math.toRadians(90));
    public Pose2d GRAB2 = new Pose2d(-19, 51.25, Math.toRadians(270));
    public Pose2d GRAB3 = new Pose2d(-19, 51.75, Math.toRadians(270));
    public Pose2d GRAB4 = new Pose2d(-19, 52.75, Math.toRadians(270));
    public Pose2d GRAB5 = new Pose2d(-19, 53, Math.toRadians(270));

    public static Pose2d PARK_CASE_1 = new Pose2d(-24, 51, Math.toRadians(90));
    public static Pose2d PARK_CASE_3 = new Pose2d(21, 51, Math.toRadians(180));
    public static Pose2d PARK_CASE_2 = new Pose2d(-3, 51, Math.toRadians(90));


    double coneCase;
    boolean gtp = false;
    boolean PPIND = false;
    boolean isDown = true;
    int cycle = 0;
    double bufferHeading = 0;
    double moveSpeed = 1.0;
    double turnSpeed = 1.0;

    //double armCycleOne = 0;
    //double armCycleTwo = 0.17;
    // double armCycleThree = 0.10;
    double armCycleFour = 0.055;
    double armCycleFive = 0.0;

    double slideHeightOne = 120;
    double slideHeightTwo = 105;
    double slideHeightThree = 70;
    double slideHeightFour = 45;

    int depositHeightPreload = 530;
    int depositHeightMid = 330;
    int depositHeightFarHigh = 550;

    double grabberCycleOne = 0.6;
    double grabberCycleTwo = 0.66;
    double grabberCycleThree = 0.64;
    double grabberCycleFour = V4B_Arm.grabberOpen;
    double grabberCycleFive = V4B_Arm.grabberClose;

    double endtime = 0;

    int numCycles = 6;
    ElapsedTime time;
    ElapsedTime timer;
    NormalizedColorSensor colorSensor;
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        time = new ElapsedTime();
        timer = new ElapsedTime();
        robot.setStartPose(new Pose2d(0, 0, 0));
        robot.arm.GrabberClose();
        robot.arm.V4BOutPose();
        robot.arm.flickerClose();
        robot.arm.write();
        robot.stopAndResetEncoders();

        boolean slidesKickout = false;

        robot.initializeWebcam();
        //robot.blueConeWebcam();
        while (!isStarted() && !isStopRequested()) {
            coneCase = robot.getConeCase();
            //coneCase = 0;
            telemetry.addData("Robot Pos", robot.getPos());
            robot.arm.grabber.servo.setPosition(V4B_Arm.grabberClose);
            robot.update();
            robot.updatePos();
            telemetry.addData("Case", coneCase);
            telemetry.update();
            timer.reset();
        }

        waitForStart();

        robot.closeCamera();

        robot.blueConeWebcam();

        time.startTime();

        while (opModeIsActive()) {
            ArrayList<CurvePoint> points = new ArrayList<>();

            switch (mRobotState) {
                case PRELOAD_STRAFE:
                    PPIND = false;
                    gtp = true;

                    points.add(new CurvePoint(new Pose2d(0, 0, 0),moveSpeed,turnSpeed,10));
                    points.add(new CurvePoint(PRE_LOAD_STRAFE,1.0,1.0,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 7){
                        newState(State.PRELOAD_MOVE);
                    }else{
                        time.reset();
                        robot.arm.GrabberClose();

                    }
                    break;

                case PRELOAD_MOVE:
                    PPIND = false;
                    gtp = false;

                    points.add(new CurvePoint(PRE_LOAD_STRAFE,1.0,1.0,10));
                    points.add(new CurvePoint(PRE_LOAD_CLEAR,1.0,1.0,10));
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,moveSpeed,turnSpeed,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.25) {
                        newState(State.PRELOAD_DEPOSIT);
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 33){
                        robot.slides.setPosition(depositHeightFarHigh);
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 35){
                        turnSpeed = 0.6;
                        moveSpeed = 0.6;
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3){
                        moveSpeed = 0.6;
                    }
                    break;

                case PRELOAD_DEPOSIT:
                    PPIND = true;
                    gtp = false;

                    points.add(new CurvePoint(RobotMovement.getLastFollowMe(),1.0,1.0,10));
                    robot.slides.setPosition(depositHeightFarHigh - 90, -0.3, 1);

                    if(time.time() > 0.2 && time.time() < 0.4) {
                        robot.arm.GrabberDeposit();
                    }

                    if(time.time() > 0.4 && time.time() < 0.5){
                        robot.arm.GrabberPartialClose();
                    }

                    if(time.time() > 0.5){
                        robot.arm.V4BAutoHold();
                    }

                    if(time.time() > 0.7) {
                        newState(State.PRELOAD_TURN);
                    }
                    break;

                case PRELOAD_TURN:
                    PPIND = true;
                    gtp = false;
                    robot.slides.setPosition(depositHeightFarHigh - 90, -0.3, 1);
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,1.0,1.0,10));
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT_TURN,1.0,1.0,10));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.0 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(5.0)) {
                        turnSpeed = 0.0;
                        newState(State.PRELOAD_FORWARD);
                    } else {
                        time.reset();
                        robot.arm.V4BFrontPose();
                        robot.arm.GrabberOpen();
                    }
                    break;

                case PRELOAD_FORWARD:
                    PPIND = true;
                    gtp = false;
                    robot.slides.setPosition(depositHeightFarHigh - 90, -0.3, 1);
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT_TURN,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_FORWARD,1.0,turnSpeed,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_FORWARD_CLEAR,1.0,turnSpeed,10));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 6.0 && Math.abs(robot.getPos().getHeading() - Math.toRadians(90)) < Math.toRadians(5)) {
                        newState(State.DRIVE_TO_INTAKE);
                    } else {
                        time.reset();
                        robot.arm.V4BFrontPose();
                        robot.arm.GrabberOpen();
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 5.0) {
                            turnSpeed = 1.0;
                        }
                    }
                    break;


                case DRIVE_TO_INTAKE:
                    gtp = false;
                    PPIND = false;
                    if (robot.slides.isDown()) {
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                        isDown = false;
                    } else {
                        robot.slides.setPower(-0.3);
                    }
                    if(isDown && robot.localizer.getPose().getX() > 25) {
                        if (robot.slides.isDown()) {
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                            isDown = false;
                        } else {
                            robot.slides.setPower(-0.3);
                        }
                    } else {
                        if (cycle == 0) {
                            robot.slides.setPosition(slideHeightOne, -0.3, 1);
                            robot.arm.manualSetPosition(0.11);
                        } else if (cycle == 1) {
                            robot.slides.setPosition(slideHeightTwo, -0.3, 1);
                            robot.arm.manualSetPosition(0.11);
                        } else if (cycle == 2) {
                            robot.slides.setPosition(slideHeightThree, -0.3, 1);
                            robot.arm.manualSetPosition(0.11);
                        } else if (cycle == 3){
                            robot.slides.setPosition(slideHeightFour, -0.3, 1);
                            robot.arm.manualSetPosition(0.11);
                        }  else if (cycle == 4){
                            robot.slides.setPosition(slideHeightFour, -0.3, 1);
                            if(robot.getPos().getX() < -13) {
                                robot.arm.manualSetPosition(0.05);
                            }else{
                                robot.arm.manualSetPosition(0.11);
                            }
                        }
                    }

                    robot.arm.GrabberOpen();
                    telemetry.addData("IS EMPTY?", LineFollowerBlue.isEmpty());


                    if(time.time() > 3.0){
                        newState(State.GRAB);
                    }

                    double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH);
                    if((Math.abs(90 - Math.toDegrees(robot.getPos().getHeading())) > 25 && Math.abs(robot.getPos().getX()) < -17) || LineFollowerBlue.isEmpty()) {
                        telemetry.addData("", "Camera failure... using odo.");
                        robot.GoTo(new Pose2d(-28, 51.25, Math.toRadians(90)), new Pose2d(0.75, 0.75, 0.75));
                    } else {
                        if(robot.getPos().getX() < -13){
                            telemetry.addLine("Here 1");
                            //robot.drive.followLine(true, 1.2, bufferHeading, distance, robot.getPos().getHeading(), 0.3, 0.3);
                            robot.drive.followLine(false, 1, VisionConstants.LineFollowerTarget, distance, LineFollowerBlue.midMaxPoint.x, 0.2, 0.2);
                            //if(Math.abs(bufferHeading - robot.getPos().getHeading()) < Math.toRadians(1.5) && Math.abs(1.2 - distance) < 0.5){
                            if(Math.abs(LineFollowerBlue.midMaxPoint.x - VisionConstants.LineFollowerTarget) < 75 && Math.abs(1 - distance) < 0.75){
                                GRAB = robot.getPos();
                                //OFFSET = new Pose2d(GRAB.getX() - OG_GRAB.getX(), GRAB.getY() - OG_GRAB.getY(), 0);
                                //robot.setOffset(OFFSET);
                                newState(State.GRAB);
                            }
                        }/*else if (robot.getPos().getX() < -18) {
                            telemetry.addLine("Here 1");
                            robot.drive.followLine(false, 1.2, VisionConstants.LineFollowerTarget, distance, LineFollower.midMaxPoint.x, 0.3, 0.3); //0.3
                            //robot.drive.followLine(true, 1.2, bufferHeading, distance, robot.getPos().getHeading(), 0.3, 0.3);
                            //if(Math.abs(bufferHeading - robot.getPos().getHeading()) < Math.toRadians(1.5) && Math.abs(1.2 - distance) < 0.5){
                        } */else if (robot.getPos().getX() < 15) { //-11 //-3
                            telemetry.addLine("Here 2");
                            bufferHeading = robot.getPos().getHeading();
                            double speed = (((robot.getPos().vec().distTo(GRAB.vec())) / 43) * 0.55) + 0.5;
                            robot.drive.followLine(false, -28, VisionConstants.LineFollowerTarget, robot.getPos().getX(), LineFollowerBlue.midMaxPoint.x, speed, speed); //0.3
                        } else {
                            telemetry.addLine("Here 3");
                            bufferHeading = robot.getPos().getHeading();
                            robot.drive.followLine(false, -28, VisionConstants.LineFollowerTarget, robot.getPos().getX(), LineFollowerBlue.midMaxPoint.x, 0.75, 0.75);
                        }
                    }

                    break;
                case GRAB:
                    gtp = true;
                    PPIND = false;
                    points.add(new CurvePoint(GRAB,0.8,0.8,10));
                    if (cycle == 0) {
                        robot.slides.setPosition(slideHeightOne - 60, -0.3, 1);
                    } else if (cycle == 1) {
                        robot.slides.setPosition(slideHeightTwo - 60, -0.3, 1);
                    } else if (cycle == 2) {
                        robot.slides.setPosition(slideHeightThree - 60, -0.3, 1);
                    }else if (cycle == 3){
                        robot.slides.setPosition(slideHeightFour - 60, -0.3, 1);
                    } else if(cycle == 4){
                        robot.slides.setPosition(slideHeightFour - 60, -0.3, 1);
                    }

                    if(time.time() > 0.1) {
                        robot.arm.GrabberClose();
                    }
                    if(time.time() > 0.4){
                        robot.arm.V4BOutPose();
                    }
                    if(time.time() > 0.6) {
                            moveSpeed = 1.0;
                            turnSpeed = 1.0;
                            if(cycle == 4 || cycle == 3){
                                newState(State.DRIVE_TO_DEPOSIT_MID);
                            } else {
                                newState(State.DRIVE_TO_DEPOSIT_HIGH_FAR);
                            }
                    }
                    break;
                case DRIVE_TO_DEPOSIT_MID:
                    gtp = false;
                    PPIND = false;
                    points.add(new CurvePoint(GRAB,1.0,1.0,10));
                    points.add(new CurvePoint(INTAKE_CLEAR,0.8,0.8,10));
                    points.add(new CurvePoint(DEPOSIT_MID,0.7,0.7,10));

                    if(time.time() > 4.0){
                        newState(State.DEPOSIT);
                    }

                    robot.slides.setPosition(depositHeightMid);

                    robot.arm.GrabberClose();

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.25) {
                         newState(State.DEPOSIT);
                    }
                    break;

                case MID_FORWARD:
                    PPIND = true;
                    gtp = false;
                    points.add(new CurvePoint(DEPOSIT_MID,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_MID_FORWARD,1.0,1.0,10));
                    points.add(new CurvePoint(INTAKE_CLEAR,1.0,1.0,10));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.5) {
                        isDown = true;
                        newState(State.DRIVE_TO_INTAKE);
                    } else {
                        time.reset();
                        robot.arm.V4BAutoHold();
                        robot.slides.setPosition(depositHeightMid - 70, -0.3, 1);
                        if (robot.slides.isDown()) {
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                            isDown = false;
                        } else {
                            robot.slides.setPower(-0.3);
                        }
                    }
                    break;
                case DRIVE_TO_DEPOSIT_HIGH_FAR:
                    PPIND = false;
                    gtp = false;
                    double slideHeight = 0;

                    points.add(new CurvePoint(GRAB,moveSpeed,turnSpeed,10));
                        points.add(new CurvePoint(DEPOSIT_HIGH_FAR_CLEAR, moveSpeed, turnSpeed,10));
                        points.add(new CurvePoint(DEPOSIT_HIGH_FAR1,moveSpeed,turnSpeed,10));


                    if(time.time() > 4.0){
                        newState(State.DEPOSIT);
                    }

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

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 35){
                        turnSpeed = 0.3;
                        moveSpeed = 0.3;
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3){
                        moveSpeed = 0.3;
                    }


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.75) {
                        newState(State.DEPOSIT);
                    }
                    break;

                case HIGH_FAR_FORWARD:
                    PPIND = true;
                    gtp = false;
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR1,moveSpeed,turnSpeed,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_FORWARD,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_FORWARD_CLEAR,1.0,1.0,10));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 5.0  && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < (cycle == 5 ? Math.toRadians(2.0) : Math.toRadians(5.0))) {
                        isDown = true;
                        if(cycle + 1 == numCycles){
                            newState(State.PARK);
                        } else {
                            newState(State.DRIVE_TO_INTAKE);
                        }
                    } else {
                        robot.arm.V4BAutoHold();
                        if(time.time() > 0.25) {
                            if (robot.slides.isDown()) {
                                time.reset();
                                robot.slides.reset();
                                robot.slides.setPower(0.0);
                                isDown = false;
                            } else {
                                robot.slides.setPower(-0.3);
                            }
                        }else{
                            robot.slides.setPower(0.0);
                        }
                    }
                    break;
                case DEPOSIT:
                    PPIND = false;
                    gtp = true;
                    points.add(new CurvePoint(RobotMovement.getLastFollowMe(), 1.0, 1.0, 10));
                    robot.slides.setPosition((cycle == 4 || cycle == 3 ? depositHeightMid-70 : depositHeightFarHigh-90), -0.3, 1);

                    if (time.time() > 0.2 && time.time() < 0.4) {
                        robot.arm.GrabberDeposit();
                    }

                    if (time.time() > 0.4 && time.time() < 0.5){
                        robot.arm.GrabberPartialClose();
                    }

                    if(time.time() > 0.5) {
                        if (cycle == 4) {
                            newState(State.PARK);
                        }else if(cycle == 3) {
                            cycle++;
                            newState(State.MID_FORWARD);
                        } else {
                            cycle++;
                            newState(State.HIGH_FAR_FORWARD);
                        }

                        if(cycle == 3 && timer.time() > 26){
                            newState(State.PARK);
                        }
                    }

                    break;
                case PARK:
                    PPIND = false;
                    gtp = true;
                    if(coneCase == 0){
                        points.add(new CurvePoint(PARK_CASE_1,1.0,1.0,15));
                    }else if(coneCase == 1){
                        points.add(new CurvePoint(PARK_CASE_2,1.0,1.0,15));
                    }else if(coneCase == 2){
                        points.add(new CurvePoint(PARK_CASE_3,1.0,1.0,15));
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2.0) {
                        newState(State.IDLE);
                    } else {
                        robot.arm.V4BHoldPos();
                        if (robot.slides.isDown()) {
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                        } else {
                            robot.slides.setPower(-0.3);
                        }
                    }
                    break;
                case IDLE:
                    telemetry.addData("TIME", endtime);
                    robot.drive.setPower(0, 0, 0);
                    robot.arm.reset();
                    robot.slides.reset();
                    robot.drive.write();
                    break;
            }

            if (points.size() != 0) {
                if(PPIND){
                    RobotMovement.followCurve(points, robot, telemetry);
                }else if(gtp){
                    robot.GoTo(points.get(points.size() - 1).toPose(), new Pose2d(points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).turnSpeed));
                }else{
                    RobotMovement.followCurveAngled(points, robot, telemetry);
                }
            } else if(mRobotState == State.DRIVE_TO_INTAKE){
                robot.drive.write();
                robot.updatePos();
            }else {
                telemetry.addLine("HERE!!!!");
                robot.drive.setPower(0, 0, 0,0);
                robot.drive.write();
                robot.updatePos();
            }


            robot.arm.write();
            robot.slides.write();
            robot.update();

            for(int i = 0; i < points.size(); i++){
                telemetry.addData("Point" + i, points.get(i).toString());
            }

            telemetry.addData("State", mRobotState);
            telemetry.addData("Position", robot.getPos());
            telemetry.addData("Cycle", cycle);
            telemetry.addData("Grabber position", robot.arm.grabber.getPosition());
            telemetry.addData("Heading Error", Math.abs(robot.getPos().getHeading() - Math.toRadians(270)));
            telemetry.addData("IsDown", isDown);
            telemetry.addData("Touch Sensor", robot.slides.isDown());
            telemetry.addData("Buffer Heading", Math.toDegrees(bufferHeading));
            telemetry.addData("FollowMe", RobotMovement.getLastFollowMe());
            telemetry.addData("Timer", timer.time());
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

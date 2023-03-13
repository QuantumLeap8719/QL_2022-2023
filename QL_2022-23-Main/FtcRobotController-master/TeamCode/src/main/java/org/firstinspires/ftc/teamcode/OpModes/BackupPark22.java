package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous(name="FULLLLSEEENDDD")
public class BackupPark22 extends LinearOpMode {

    private enum State {
        DRIVE_TO_DEPOSIT_PRELOAD,
        DRIVE_TO_INTAKE,
        GRAB,
        DRIVE_TO_DEPOSIT_MID,
        DRIVE_TO_DEPOSIT_HIGH_FAR,
        DEPOSIT,
        PARK,
        IDLE
    }

    State mRobotState = State.DRIVE_TO_DEPOSIT_PRELOAD;

    public Pose2d PRE_LOAD_CLEAR = new Pose2d(6, -15, Math.toRadians(0));
    public Pose2d PRE_LOAD_DEPOSIT = new Pose2d(-1.1, -28.25, Math.toRadians(24.190));

    public Pose2d INTAKE_CLEAR = new Pose2d(12, -52, Math.toRadians(90));
    public Pose2d INTAKE_FAR_CLEAR = new Pose2d(-13, -52, Math.toRadians(90));
    public Pose2d BACk_FAR_CLEAR = new Pose2d(-13, -52, Math.toRadians(117));
    public Pose2d DEPOSIT_HIGH_FAR_CLEAR = new Pose2d(-7.5, -54, Math.toRadians(90));

    public Pose2d DEPOSIT_HIGH = new Pose2d(1, -49, Math.toRadians(65));
    public Pose2d DEPOSIT_MID = new Pose2d(4.5, -46, Math.toRadians(117));
    public static Pose2d DEPOSIT_HIGH_FAR = new Pose2d(-21.25, -45.25, Math.toRadians(117)); //SECOND HIGH
    //public Pose2d DEPOSIT_HIGH_FAR = new Pose2d(-19.5, -43.5, Math.toRadians(113));

    public Pose2d GRAB = new Pose2d(28.25, -51.25, Math.toRadians(90));
    public Pose2d GRAB2 = new Pose2d(28.25, -51.25, Math.toRadians(90));
    public Pose2d GRAB3 = new Pose2d(28.25, -51.75, Math.toRadians(90));
    public Pose2d GRAB4 = new Pose2d(28.15, -52.75, Math.toRadians(90));
    public Pose2d GRAB5 = new Pose2d(28.15, -53, Math.toRadians(90));

    public static Pose2d PARK_CASE_1 = new Pose2d(28, -25, Math.toRadians(90));
    public static Pose2d PARK_CASE_3 = new Pose2d(-18.7, -25, Math.toRadians(90));
    public static Pose2d PARK_CASE_2 = new Pose2d(3.2, -25, Math.toRadians(90));

    double coneCase;
    boolean gtp = false;
    int cycle = 0;

    //double armCycleOne = 0;
    //double armCycleTwo = 0.17;
   // double armCycleThree = 0.10;
    double armCycleFour = 0.055;
    double armCycleFive = 0.0;

    double slideHeightOne = 62;
    double slideHeightTwo = 32;
    double slideHeightThree = 0;
    double slideHeightFour = 0;

    int depositHeightPreload = 390;
    int depositHeightMid = 385;
    int depositHeightFarHigh = 580;

    double grabberCycleOne = 0.6;
    double grabberCycleTwo = 0.66;
    double grabberCycleThree = 0.64;
    double grabberCycleFour = V4B_Arm.grabberOpen;
    double grabberCycleFive = V4B_Arm.grabberClose;

    int numCycles = 5;
    ElapsedTime time;
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();

        robot.localizer.reset();
        robot.setStartPose(new Pose2d(0, 0, 0));

        robot.arm.GrabberClose();
        robot.arm.V4BAutoHold();
        robot.arm.write();

        boolean slidesKickout = false;

        robot.initializeWebcam();
        while (!isStarted() && !isStopRequested()) {
            coneCase = robot.getConeCase();
            //coneCase = 0;
            telemetry.addData("Case", coneCase);
            telemetry.update();
        }


        robot.stopWebcam();

        waitForStart();

        time.startTime();

        while (opModeIsActive()) {
            ArrayList<CurvePoint> points = new ArrayList<>();

            switch (mRobotState) {
                case DRIVE_TO_DEPOSIT_PRELOAD:
                    points.add(new CurvePoint(new Pose2d(0, 0, 0),0.4,0.4,10));
                    points.add(new CurvePoint(PRE_LOAD_CLEAR,0.4,0.4,10));
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,0.4,0.4,10));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0 && slidesKickout && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1)) {
                        if(time.time() > 0.3){
                            robot.slides.setPosition(depositHeightPreload - 50, -0.3, 1);
                        }

                        if(time.time() > 0.5 && time.time() < 0.75) {
                            robot.arm.GrabberOpen();
                        }

                        if(time.time() > 0.75 && time.time() < 0.85){
                            robot.arm.GrabberClose();
                        }

                        if(time.time() > 0.85 && time.time() < 1.15){
                            robot.arm.V4BFrontPose();
                        }

                        if(time.time() > 1.15) {
                            slidesKickout = false;
                            newState(State.PARK);
                        }
                    }else{
                        time.reset();
                        robot.arm.V4BOutPose();
                        robot.arm.GrabberClose();
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 32.5){
                            robot.slides.setPosition(depositHeightPreload);
                        }

                        if(Math.abs(robot.slides.getPosition() - depositHeightPreload) < 20){
                            slidesKickout = true;
                        }
                    }
                    break;
                case DRIVE_TO_INTAKE:
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,1.0,1.0,10));

                    if(cycle >= 2){
                        points.add(new CurvePoint(BACk_FAR_CLEAR, 1.0, 1.0, 10));
                    }else {
                        points.add(new CurvePoint(INTAKE_CLEAR, 1.0, 1.0, 10));
                    }

                    double speed = 1.0;

                    if (robot.getPos().vec().distTo(GRAB.vec()) < 30) {
                        speed = 0.5;
                    }

                    if(cycle == 0) {
                        points.add(new CurvePoint(GRAB, speed, speed, 10));
                    } else if(cycle == 1){
                        points.add(new CurvePoint(GRAB2, speed, speed, 10));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(GRAB3, speed, speed, 10));
                    } else if (cycle == 3){
                        points.add(new CurvePoint(GRAB4, speed, speed, 10));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(GRAB5, speed, speed, 10));
                    }else {
                        points.add(new CurvePoint(GRAB2, speed, speed, 10));
                    }

                    if(cycle==0){
                        robot.slides.setPosition(slideHeightOne, -0.247, 1);
                    }else if(cycle==1){
                        robot.slides.setPosition(slideHeightTwo, -0.247, 1);
                    }else if(cycle==2){
                        robot.slides.setPosition(slideHeightThree, -0.25, 1);
                    } else {
                        robot.slides.setPosition(slideHeightFour, -0.25, 1);
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
                    if(cycle==0){
                        robot.slides.setPosition(slideHeightOne, -0.35, 1.0);
                    }else if(cycle==1){
                        robot.slides.setPosition(slideHeightTwo,-0.35, 1.0);
                    }else if(cycle==2){
                        robot.slides.setPosition(slideHeightThree,-0.35, 1.0);
                    }else{
                        if (robot.slides.isDown()) {
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                        } else {
                            robot.slides.setPower(-0.4);
                        }
                    }

                    if(cycle == 0) {
                        points.add(new CurvePoint(GRAB, 1.0, 1.0, 10));
                    } else if(cycle == 1){
                        points.add(new CurvePoint(GRAB2, 1.0, 1.0, 10));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(GRAB3, 1.0, 1.0, 10));
                    } else if (cycle == 3){
                        points.add(new CurvePoint(GRAB4, 1.0, 1.0, 10));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(GRAB5, 1.0, 1.0, 10));
                    }else {
                        points.add(new CurvePoint(GRAB2, 1.0, 1.0, 10));
                    }

                    if(time.time() > 0.15 && time.time() < 0.4){
                        if(cycle == 0 || cycle == 1) {
                            robot.arm.grabberPos(grabberCycleFive);
                        }else if(cycle == 2){
                            robot.arm.grabberPos(grabberCycleFive);
                        } else if (cycle == 3){
                            robot.arm.grabberPos(grabberCycleFive);
                        } else if (cycle == 4) {
                            robot.arm.grabberPos(grabberCycleFive);
                        }
                    }

                   if(time.time() > 0.01 && time.time() < 0.15){
                        if(cycle == 0 || cycle == 1) {
                            robot.arm.manualSetPosition(armCycleFour);
                        }else if(cycle == 2){
                            robot.arm.manualSetPosition(armCycleFour);
                        } else if (cycle == 3){
                            robot.arm.manualSetPosition(armCycleFour);
                        } else if (cycle == 4) {
                            robot.arm.manualSetPosition(armCycleFive);
                        }
                    }

                    if(time.time() > 0.4){
                        robot.arm.V4BOutPose();
                    }

                    if(time.time() > 0.8) {
                        if(cycle == 0) {
                            newState(State.DRIVE_TO_DEPOSIT_MID);
                        } else {
                            newState(State.DRIVE_TO_DEPOSIT_HIGH_FAR);
                        }
                    }

                    break;
                case DRIVE_TO_DEPOSIT_MID:
                    points.add(new CurvePoint(GRAB,1.0,1.0,10));
                    points.add(new CurvePoint(INTAKE_CLEAR,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_MID,1.0,1.0,10));

                    robot.slides.setPosition(depositHeightMid);

                    robot.arm.GrabberClose();

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1)) {
                        newState(State.DEPOSIT);
                    }
                    break;
                case DRIVE_TO_DEPOSIT_HIGH_FAR:
                    double slideHeight = 0;

                    points.add(new CurvePoint(GRAB,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_CLEAR,0.7,0.7,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR,0.3,0.3,10));

                    robot.arm.GrabberClose();

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 25) {
                        if(cycle == 2){
                            slideHeight = depositHeightFarHigh - 10;
                        } else {
                            slideHeight = depositHeightFarHigh;
                        }
                        robot.slides.setPosition(slideHeight);
                    }else{
                        robot.slides.setPower(0);
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1.2) && Math.abs(robot.slides.getPosition() - slideHeight) <= 20) {
                        newState(State.DEPOSIT);
                    }
                    break;
                case DEPOSIT:
                   if(cycle == 0) {
                       points.add(new CurvePoint(DEPOSIT_MID, 1.0, 1.0, 10));
                   } else {
                       points.add(new CurvePoint(DEPOSIT_HIGH_FAR,1.0,1.0,10));
                   }

                   if (time.time() > 0.3) {
                       robot.slides.setPosition((cycle == 0 ? depositHeightMid : depositHeightFarHigh) - 50, -0.3, 1);
                   }else{
                       robot.slides.setPosition(depositHeightFarHigh);
                   }

                   if (time.time() > 0.5 && time.time() < 0.7) {
                       robot.arm.GrabberOpen();
                   }

                   if (time.time() > 0.7 && time.time() < 0.75) {
                       robot.arm.GrabberClose();
                   }

                   if (time.time() > 0.75 && time.time() < 1.0) {
                       robot.arm.V4BFrontPose();
                   }

                   if (time.time() > 1.25) {
                       if (robot.slides.isDown()) {
                           robot.slides.reset();
                           robot.slides.setPower(0.0);
                       } else {
                           robot.slides.setPower(-0.26);
                       }

                       if((cycle+1) == numCycles){
                           newState(State.PARK);
                       }else {
                           cycle++;
                           newState(State.DRIVE_TO_INTAKE);
                       }
                   }
                   break;
                case PARK:
                    gtp = true;
                    if(coneCase == 0){
                        points.add(new CurvePoint(PARK_CASE_1,0.4,0.4,15));
                    }else if(coneCase == 1){
                        points.add(new CurvePoint(PARK_CASE_2,0.4,0.4,15));
                    }else if(coneCase == 2){
                        points.add(new CurvePoint(PARK_CASE_3,0.4,0.4,15));
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
                    break;
            }

            if (points.size() != 0) {
                if (!gtp) {
                    RobotMovement.followCurve(points, robot, telemetry);
                } else {
                    robot.GoTo(points.get(points.size() - 1).toPose(), new Pose2d(points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).turnSpeed));
                }
            } else {
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

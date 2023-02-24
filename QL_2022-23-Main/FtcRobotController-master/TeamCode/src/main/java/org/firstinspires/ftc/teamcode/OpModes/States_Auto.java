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

@Autonomous(name="OhioAuto")
public class States_Auto extends LinearOpMode {

    private enum State {
        DRIVE_TO_DEPOSIT_PRELOAD,
        DRIVE_TO_INTAKE,
        GRAB,
        DRIVE_TO_DEPOSIT_MID,
        DRIVE_TO_DEPOSIT_HIGH_FAR,
        DEPOSIT,
        PARK
    }

    State mRobotState = State.DRIVE_TO_DEPOSIT_PRELOAD;

    public Pose2d PRE_LOAD_CLEAR = new Pose2d(6, -27.4, Math.toRadians(0));
    public Pose2d PRE_LOAD_CLEAR2 = new Pose2d(3.3, -46.5, Math.toRadians(0));
    public Pose2d PRE_LOAD_DEPOSIT = new Pose2d(-4, -51.2, Math.toRadians(24.190));

    public Pose2d INTAKE_CLEAR = new Pose2d(12, -52, Math.toRadians(90));
    public Pose2d INTAKE_FAR_CLEAR = new Pose2d(-13, -52, Math.toRadians(90));
    public Pose2d DEPOSIT_HIGH_FAR_CLEAR = new Pose2d(-6.3, -52.1, Math.toRadians(90));

    public Pose2d DEPOSIT_HIGH = new Pose2d(1, -49, Math.toRadians(65));
    public Pose2d DEPOSIT_MID = new Pose2d(4, -45, Math.toRadians(117));
    public static Pose2d DEPOSIT_HIGH_FAR = new Pose2d(-22.5, -49, Math.toRadians(132)); //SECOND HIGH
    //public Pose2d DEPOSIT_HIGH_FAR = new Pose2d(-19.5, -43.5, Math.toRadians(113));

    public Pose2d GRAB = new Pose2d(28, -51.5, Math.toRadians(90));
    public Pose2d GRAB2 = new Pose2d(28, -51.75, Math.toRadians(90));
    public Pose2d GRAB3 = new Pose2d(28, -51.5, Math.toRadians(90));
    public Pose2d GRAB4 = new Pose2d(28, -51.25, Math.toRadians(90));
    public Pose2d GRAB5 = new Pose2d(28, -51.0, Math.toRadians(90));

    public static Pose2d PARK_CASE_1 = new Pose2d(24, -47, Math.toRadians(90));
    public static Pose2d PARK_CASE_3 = new Pose2d(-18.7, -50.25, Math.toRadians(90));
    public static Pose2d PARK_CASE_2 = new Pose2d(3.2, -50.25, Math.toRadians(90));

    double coneCase;
    boolean gtp = false;
    int cycle = 0;

    double armCycleOne = 0;
    double armCycleTwo = 0.17;
    double armCycleThree = 0.10;
    double armCycleFour = 0.055;
    double armCycleFive = 0;

    double slideHeightOne = 85;
    double slideHeightTwo = 55;
    double slideHeightThree = 28;
    double slideHeightFour = 5;

    double depositHeightPreload = 565;
    double depositHeightMid = 400;
    double depositHeightFarHigh = 575;

    double grabberCycleOne = 0.6;
    double grabberCycleTwo = 0.66;
    double grabberCycleThree = 0.64;
    double grabberCycleFour = V4B_Arm.grabberOpen;
    double grabberCycleFive = V4B_Arm.grabberOpen;

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

        robot.initializeWebcam();
        while (!isStarted() && !isStopRequested()) {
            coneCase = robot.getConeCase();
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
                    points.add(new CurvePoint(new Pose2d(0, 0, 0),1.0,1.0,10));
                    points.add(new CurvePoint(PRE_LOAD_CLEAR,1.0,1.0,10));
                    points.add(new CurvePoint(PRE_LOAD_CLEAR2,1.0,1.0,10));
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,1.0,1.0,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1)) {
                        robot.slides.setPosition(depositHeightPreload - 50, -0.3, 1);
                        if(time.time() > 0.2) {
                            robot.arm.GrabberPartial();
                        }

                        if(time.time() > 0.4){
                            robot.arm.GrabberClose();
                        }

                        if(time.time() > 0.5){
                            robot.arm.V4BFrontPose();
                        }

                        if(time.time() > 0.7) {
                            robot.arm.GrabberOpen();
                            newState(State.DRIVE_TO_INTAKE);
                        }
                    }else{
                        time.reset();
                        robot.arm.V4BOutPose();
                        robot.arm.GrabberClose();
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 30){
                            robot.slides.setPosition(depositHeightPreload);
                        }
                    }
                    break;
                case DRIVE_TO_INTAKE:
                    points.add(new CurvePoint(PRE_LOAD_DEPOSIT,1.0,1.0,10));
                    if(cycle >= 2){
                        points.add(new CurvePoint(INTAKE_FAR_CLEAR, 1.0, 1.0, 10));
                    }else {
                        points.add(new CurvePoint(INTAKE_CLEAR, 1.0, 1.0, 10));
                    }
                    if(cycle == 0) {
                        points.add(new CurvePoint(GRAB, 0.7, 1.0, 15));
                    } else if(cycle == 1){
                        points.add(new CurvePoint(GRAB2, 0.7, 1.0, 15));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(GRAB3, 0.7, 1.0, 15));
                    } else if (cycle == 3){
                        points.add(new CurvePoint(GRAB4, 0.7, 1.0, 15));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(GRAB5, 0.7, 1.0, 15));
                    }else {
                        points.add(new CurvePoint(GRAB2, 0.7, 1.0, 15));
                    }

                    if(cycle==0){
                        robot.slides.setPosition(slideHeightOne, -0.2501, 1);
                    }else if(cycle==1){
                        robot.slides.setPosition(slideHeightTwo, -0.2501, 1);
                    }else if(cycle==2){
                        robot.slides.setPosition(slideHeightThree, -0.2501, 1);
                    }else{
                        if(robot.slides.isDown()){
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                        } else {
                            robot.slides.setPower(-0.2501);
                        }
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.5 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1.5)) {
                        newState(State.GRAB);
                    }else{
                        if(cycle == 0 || cycle == 1) {
                            robot.arm.manualSetPosition(armCycleFour);
                            robot.arm.grabberPos(grabberCycleFour);
                        }/* else if(cycle == 1){
                            robot.arm.manualSetPosition(armCycleTwo);
                            robot.arm.grabberPos(grabberCycleTwo);
                            //robot.slides.setPosition(85, -0.2501, 1);
                        } */else if(cycle == 2){
                            robot.arm.manualSetPosition(armCycleFour);
                            robot.arm.grabberPos(grabberCycleFour);
                        } else if (cycle == 3){
                            robot.arm.manualSetPosition(armCycleFour);
                            robot.arm.grabberPos(grabberCycleFour);
                        } else if (cycle == 4){
                            robot.arm.manualSetPosition(armCycleFive);
                            robot.arm.grabberPos(grabberCycleFive);
                        }
                        time.reset();
                    }
                    break;
                case GRAB:
                    if(cycle==0){
                        robot.slides.setPosition(slideHeightOne, -0.2501, 1);
                    }else if(cycle==1){
                        robot.slides.setPosition(slideHeightTwo, -0.2501, 1);
                    }else if(cycle==2){
                        robot.slides.setPosition(slideHeightThree, -0.2501, 1);
                    }else{
                        if(robot.slides.isDown()){
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                        } else {
                            robot.slides.setPower(-0.2501);
                        }
                    }

                    if(cycle == 0) {
                        points.add(new CurvePoint(GRAB, 1.0, 1.0, 15));
                    } else if(cycle == 1){
                        points.add(new CurvePoint(GRAB2, 1.0, 1.0, 15));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(GRAB3, 1.0, 1.0, 15));
                    } else if (cycle == 3){
                        points.add(new CurvePoint(GRAB4, 1.0, 1.0, 15));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(GRAB5, 1.0, 1.0, 15));
                    }else {
                        points.add(new CurvePoint(GRAB2, 1.0, 1.0, 15));
                    }

                    robot.arm.GrabberClose();

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

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1)) {
                        newState(State.DEPOSIT);
                    }
                    break;
                case DRIVE_TO_DEPOSIT_HIGH_FAR:
                    points.add(new CurvePoint(GRAB,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR_CLEAR,1.0,1.0,10));
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR,1.0,1.0,10));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 35) {
                        robot.slides.setPosition(depositHeightFarHigh);
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < Math.toRadians(1)) {
                        newState(State.DEPOSIT);
                    }
                    break;
                case DEPOSIT:
                   if(cycle == 0) {
                           points.add(new CurvePoint(DEPOSIT_MID, 1.0, 1.0, 10));
                   } else {
                            points.add(new CurvePoint(DEPOSIT_HIGH_FAR,1.0,1.0,10));
                    }

                    //if (time.time() > 0.7) {
                        robot.slides.setPosition((cycle == 0 ? depositHeightMid : depositHeightFarHigh) - 50, -0.3, 1);
                    //} else {
                     //   robot.slides.setPosition(cycle == 0 ? 370 : 555);
                    //}
                    if (time.time() > 0.2) {
                        robot.arm.GrabberPartial();
                    }

                    if (time.time() > 0.4) {
                        robot.arm.GrabberClose();
                    }

                    if (time.time() > 0.5) {
                        robot.arm.V4BFrontPose();
                    }

                    if (time.time() > 0.7) {
                        if (robot.slides.isDown()) {
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                        } else {
                            robot.slides.setPower(-0.2501);
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
                    points.add(new CurvePoint(DEPOSIT_HIGH_FAR,1.0,1.0,10));
                    if(coneCase == 0){
                        points.add(new CurvePoint(PARK_CASE_1,1.0,1.0,10));
                    }else if(coneCase == 1){
                        points.add(new CurvePoint(PARK_CASE_2,1.0,1.0,10));
                    }else if(coneCase == 2){
                        points.add(new CurvePoint(PARK_CASE_3,1.0,1.0,10));
                    }
                    robot.arm.V4BFrontHoldPos();
                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.2501);
                    }
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

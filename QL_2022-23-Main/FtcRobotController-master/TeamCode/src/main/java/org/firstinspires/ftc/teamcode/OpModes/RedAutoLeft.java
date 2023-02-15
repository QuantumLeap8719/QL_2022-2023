package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

//@Autonomous(name="LeftHigh")
public class RedAutoLeft extends LinearOpMode {

    private enum State {
        CLEAR,
        STRAFE,
        TRAVEL_TO_DEPOSITONE,
        TURN_AT_DEPOSIT_ONE,
        DEPOSIT_ONE,
        LIFT_SLIDES,
        DEPOSIT_TWO,
        DEPOSIT_THREE,
        TRAVEL_TO_STONE_PICKUP,
        TRAVEL_TO_STONE_PICKUPTWO,
        RETURN_TO_STONE_PICKUP,
        RETURN_TO_STONE_PICKUPTWO,
        TRAVEL_TO_DEPOSITTWO,
        TRAVEL_TO_DEPOSITTHREE,
        GRAB_STONE,
        GRAB_STONE_2,
        CASE_0,
        CASE_0_TURN,
        CASE_0_STRAFE,
        CASE_1,
        CASE_2,
        CASE_2_TURN,
        CASE_2_STRAFE,
        PARK
    }

    State mRobotState = State.CLEAR;

    double coneCase;
    boolean gtp = false;
    int cycle = 0;
    double slideHeight1;
    double slideHeight2;
    double slideHeight3;
    double slideHeight4 = 0;
    double topGoalPos;

    double armCycleOne = 0;
    double armCycleTwo = 0.15;
    double armCycleThree = 0.09;
    double armCycleFour = 0.04;
    double armCycleFive = 0;

    double grabberCycleOne = 0.6;
    double grabberCycleTwo = 0.66;
    double grabberCycleThree = 0.64;
    double grabberCycleFour = 0.6;
    double grabberCycleFive = 0.6;

    int numCycles = 5;
    ElapsedTime time;
    Robot robot;
    double clearance_dist = 0.0;

    public static Pose2d CLEAR = new Pose2d(6 , -42, Math.toRadians(0));
    public static Pose2d START = new Pose2d(0 , 0, 0);

    //DEPOSIT
    public static Pose2d STRAFE = new Pose2d(-4, -52.5, Math.toRadians(35));

    public static Pose2d ONE_TWO_DEPOSIT = new Pose2d(-4, -52.5, Math.toRadians(35));
    public static Pose2d THREE_DEPOSIT = new Pose2d(-3.5, -52.5, Math.toRadians(35));
    public static Pose2d FOUR_FIVE_DEPSOIT = new Pose2d(-3.5, -52.5, Math.toRadians(35));
    public static Pose2d TURN_AT_DEPOSIT_ONE = new Pose2d(5, -44, Math.toRadians(90));
    public static Pose2d BACK_AT_DEPOSIT_ONE = new Pose2d(5, -48, Math.toRadians(90));
    public static Pose2d DEPOSIT_ONE = new Pose2d(5.5, -42.25, Math.toRadians(93));
    public static Pose2d TRAVEL_TO_STONEPICKUP = new Pose2d(-1.5, -54.2, Math.toRadians(45));

    public static Pose2d GRAB_STONE = new Pose2d(26, -50, Math.toRadians(90));
    public static Pose2d GRAB_STONE2 = new Pose2d(27, -49.5, Math.toRadians(90));
    public static Pose2d GRAB_STONE3 = new Pose2d(27.5, -48.5, Math.toRadians(90));
    public static Pose2d GRAB_STONE4 = new Pose2d(27.1, -47, Math.toRadians(90));
    public static Pose2d GRAB_STONE5 = new Pose2d(27.1, -46.75, Math.toRadians(90));

    public static Pose2d GRAB_STONE6 = new Pose2d(34.5, -47, Math.toRadians(90));


    public static Pose2d DEPOSIT_TWO = new Pose2d(0.6, -39, Math.toRadians(90));
    public static Pose2d DEPOSIT_THREE = new Pose2d(0.6, -38, Math.toRadians(90));

    public static Pose2d CASE_3_MID = new Pose2d(-27, -53, Math.toRadians(0));
    public static Pose2d CASE_1 = new Pose2d(28.5, -47, Math.toRadians(0));
    public static Pose2d CASE_3 = new Pose2d(-18.7, -31, Math.toRadians(0));
    public static Pose2d CASE_2_TURN = new Pose2d(3.2, -28.4, Math.toRadians(0));

    public static Pose2d PARK_1 = new Pose2d(0,0,0);
    public static Pose2d PARK_2 = new Pose2d(0,0,0);
    public static Pose2d PARK_3 = new Pose2d(0,0,0);



    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();

        robot.localizer.reset();
        robot.setStartPose(new Pose2d(-0.687, 2, 0));

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
                case CLEAR:
                    points.add(new CurvePoint(START,1.0,1.0,15));
                    points.add(new CurvePoint(CLEAR,1.0,1.0,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 10){
                        robot.slides.setPosition(625);
                    }

                    if((robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 10)) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.arm.V4BOutPose();
                    } else {
                        newState(State.STRAFE);
                    }
                    break;
                case STRAFE:
                    points.add(new CurvePoint(CLEAR,1,1,4));
                    points.add(new CurvePoint(STRAFE,0.6,1,4));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 1.5 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) >= Math.toRadians(2)) {
                        time.reset();
                        robot.arm.V4BOutPose();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(625);
                    } else {
                        if(time.time() > 1.2) {
                            robot.slides.setPosition(580, -0.3, 1);
                        } else {
                            robot.slides.setPosition(625);
                        }
                        if(time.time() > 1.4) {
                            robot.arm.GrabberPartial();
                        }

                        if(time.time() > 1.6){
                            robot.arm.GrabberClose();
                        }

                        if(time.time() > 1.7){
                            robot.arm.V4BFrontPose();
                        }

                        if(time.time() > 2) {
                            robot.arm.GrabberOpen();
                            if(robot.slides.isDown()){
                                robot.slides.reset();
                                robot.slides.setPower(0.0);
                            } else {
                                robot.slides.setPower(-0.2501);
                            }
                        }

                        if(time.time() > 2.2){
                            newState(State.TURN_AT_DEPOSIT_ONE);
                        }

                    }
                    break;

                case TURN_AT_DEPOSIT_ONE:
                    if(cycle == 0) {
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE, 0.6, 0.5, 6));
                    } else if(cycle == 1){
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE2, 0.6, 0.5, 6));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE3, 0.6, 0.5, 6));
                    } else if (cycle == 3){
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE4, 0.6, 0.5, 6));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE5, 0.6, 0.5, 6));
                    }else {
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE6, 0.6, 0.5, 6));
                    }

                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.2501);
                    }


           //ARM TUNING AREA

                    if(cycle == 0){
                        robot.slides.setPosition(125, -0.2501, 1);
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 1.5) {
                        time.reset();
                        if(cycle == 0) {
                            robot.arm.manualSetPosition(armCycleOne);
                            robot.arm.grabberPos(grabberCycleOne);
                        } else if(cycle == 1){
                            robot.arm.manualSetPosition(armCycleTwo);
                            robot.arm.grabberPos(grabberCycleTwo);
                            //robot.slides.setPosition(85, -0.2501, 1);
                            //90 for 2nd Cone //60 for 3rd Cone // 25 for 4th Cone
                        } else if(cycle == 2){
                            robot.arm.manualSetPosition(armCycleThree);
                            robot.arm.grabberPos(grabberCycleThree);
                            //robot.slides.setPosition(55, -0.2501, 1);
                        } else if (cycle == 3){
                            robot.arm.manualSetPosition(armCycleFour);
                            robot.arm.grabberPos(grabberCycleFour);
                            //robot.slides.setPosition(25, -0.2501, 1);
                        } else if (cycle == 4){
                            robot.arm.manualSetPosition(armCycleFive);
                            robot.arm.grabberPos(grabberCycleFive);
                            //robot.slides.setPosition(0, -0.2501, 1);
                        }
                    } else {
                        robot.arm.GrabberClose();
                        if (time.time() > 0.4) {
                            newState(State.LIFT_SLIDES);
                        }
                    }
                    break;

                case LIFT_SLIDES:
                    if(cycle == 0) {
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE, 1.0, 1.0, 15));
                    } else if(cycle == 1){
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE2, 1.0, 1.0, 15));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE3, 1.0, 1.0, 15));
                    } else if (cycle == 3){
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE4, 1.0, 1.0, 15));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE5, 1.0, 1.0, 15));
                    }else {
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE6, 1.0, 1.0, 15));
                    }

                    robot.arm.GrabberClose();
                    robot.arm.V4BOutPose();
                    if(time.time() > 0.3) {
                        newState(State.DEPOSIT_ONE);
                    }
                    break;





            //STACK TO DEPOSIT
                case DEPOSIT_ONE:
                    if(cycle == 0 || cycle == 1) {
                        points.add(new CurvePoint(GRAB_STONE, 1.0, 1.0, 15));
                        if(robot.getPos().vec().distTo(BACK_AT_DEPOSIT_ONE.vec()) > 15) {
                            points.add(new CurvePoint(BACK_AT_DEPOSIT_ONE, 0.75, 1.0, 4));
                            points.add(new CurvePoint(ONE_TWO_DEPOSIT, 0.75, 1.0, 4));
                        }else{
                            points.add(new CurvePoint(BACK_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                            points.add(new CurvePoint(ONE_TWO_DEPOSIT, 1.0, 1.0, 15));
                        }
                    } else if(cycle == 2){
                        points.add(new CurvePoint(GRAB_STONE, 1.0, 1.0, 15));
                        if(robot.getPos().vec().distTo(BACK_AT_DEPOSIT_ONE.vec()) > 15) {
                            points.add(new CurvePoint(BACK_AT_DEPOSIT_ONE, 0.75, 1.0, 4));
                            points.add(new CurvePoint(THREE_DEPOSIT, 0.75, 1.0, 4));
                        }else{
                            points.add(new CurvePoint(BACK_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                            points.add(new CurvePoint(THREE_DEPOSIT, 1.0, 1.0, 15));
                        }
                } else if (cycle == 3 || cycle == 4 || cycle == 5){
                        points.add(new CurvePoint(GRAB_STONE, 1.0, 1.0, 15));
                        if(robot.getPos().vec().distTo(BACK_AT_DEPOSIT_ONE.vec()) > 15) {
                            points.add(new CurvePoint(BACK_AT_DEPOSIT_ONE, 0.75, 1.0, 4));
                            points.add(new CurvePoint(FOUR_FIVE_DEPSOIT, 0.75, 1.0, 4));
                        }else{
                            points.add(new CurvePoint(BACK_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                            points.add(new CurvePoint(FOUR_FIVE_DEPSOIT, 1.0, 1.0, 15));
                        }
                    }



                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) >= Math.toRadians(1)) {
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 22){
                            robot.slides.setPosition(640);
                        }
                        time.reset();
                        robot.arm.GrabberClose();
                    } else {
                        if(time.time() > 0.7) {
                            robot.slides.setPosition(580, -0.3, 1);
                        } else {
                            robot.slides.setPosition(640);
                        }
                        if (time.time() > 0.9) {
                            robot.arm.GrabberPartial();
                        }

                        if (time.time() > 1.1) {
                            robot.arm.GrabberClose();
                        }

                        if (time.time() > 1.2) {
                            robot.arm.V4BFrontPose();
                        }

                        if (time.time() > 1.4) {
                            if(robot.slides.isDown()){
                                robot.slides.reset();
                                robot.slides.setPower(0.0);
                            } else {
                                robot.slides.setPower(-0.2501);
                            }
                            cycle++;
                            if (cycle < 5) {
                                newState(State.TURN_AT_DEPOSIT_ONE);
                            } else {
                                if(coneCase == 0){
                                    newState(State.CASE_0);
                                } else if (coneCase == 1){
                                    newState(State.CASE_1);
                                } else {
                                    newState(State.CASE_2);
                                }
                            }
                        }
                    }
                    break;

                case CASE_0:
                    points.add(new CurvePoint(STRAFE,1.0,1.0,15));
                    points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE,1.0,1.0,15));
                    points.add(new CurvePoint(CASE_1,1.0,1.0,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.V4BFrontHoldPos();
                        if(robot.slides.isDown()){
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                        } else {
                            robot.slides.setPower(-0.2501);
                        }
                        robot.arm.GrabberClose();
                    } else {
                        if(time.time() > 0.2){
                            newState(State.PARK);
                        }
                    }
                    break;

                case CASE_1:
                    points.add(new CurvePoint(STRAFE,0.1,0.1,15));
                    points.add(new CurvePoint(CASE_2_TURN,0.1,0.1,15));

                    if(Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) > Math.toRadians(3)) {
                        time.reset();
                        robot.arm.V4BFrontHoldPos();
                        if(robot.slides.isDown()){
                            robot.slides.reset();
                            robot.slides.setPower(0.0);
                        } else {
                            robot.slides.setPower(-0.2501);
                        }
                    }  else {
                        if(time.time() > 0.2){
                            newState(State.PARK);
                        }
                    }
                    break;

                case CASE_2:
                    points.add(new CurvePoint(STRAFE,1.0,1.0,10));
                    points.add(new CurvePoint(CASE_3_MID,1.0,1.0,10));
                    points.add(new CurvePoint(CASE_3,1.0,1.0,10));
                    robot.arm.V4BFrontHoldPos();
                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.2501);
                    }
                    break;
                case PARK:
                    robot.drive.setPower(0,0,0,0);
                    break;
            }

            if(mRobotState != State.PARK) {
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
            }
            robot.arm.write();
            robot.slides.write();
            robot.update();

            for(int i = 0; i < points.size(); i++){
                telemetry.addData("Point" + i, points.get(i) );
            }

            telemetry.addData("State", mRobotState);
            telemetry.addData("Position", robot.getPos());
            telemetry.addData("Clearance Dist", clearance_dist);
            telemetry.addData("Cycle", cycle);
            telemetry.update();
        }
    }

    public void newState (State state){
        time.reset();
        mRobotState = state;
    }
}
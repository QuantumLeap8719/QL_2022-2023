package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Vision.BlueSleeveDetector;

import java.util.ArrayList;

@Autonomous(name="Park")
public class Park extends LinearOpMode {

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
    double armCycleTwo = 0.17;
    double armCycleThree = 0.105;
    double armCycleFour = 0.055;
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

    public static Pose2d CLEAR = new Pose2d(-5.75, -42, Math.toRadians(0));
    public static Pose2d START = new Pose2d(0 , 0, 0);

    //DEPOSIT
    public static Pose2d STRAFE = new Pose2d(2.5, -50.5, Math.toRadians(-35));



    public static Pose2d MID_DEPOSIT = new Pose2d(0.5, -43.75, Math.toRadians(-125)); //MID
    public static Pose2d BACKUP_HIGHDEPOSIT = new Pose2d(25, -42, Math.toRadians(-132)); //SECOND HIGH
    public static Pose2d BACKUP_HIGHDEPOSIT_4_5 = new Pose2d(21.5, -42, Math.toRadians(-132)); //SECOND HIGH
    //public static Pose2d FOUR_FIVE_DEPSOIT = new Pose2d(-3.5, -52.5, Math.toRadians(35));
    public static Pose2d TURN_AT_DEPOSIT_ONE = new Pose2d(-5, -46.5, Math.toRadians(-90));
    public static Pose2d MID_CLEAR = new Pose2d(-10, -50, Math.toRadians(-90));
    public static Pose2d BACK_AT_DEPOSIT_ONE = new Pose2d(-6, -54, Math.toRadians(-90));
    public static Pose2d CURVE_FORBACKUPHIGH = new Pose2d(9, -50, Math.toRadians(-90)); //Curve
    public static Pose2d TRAVEL_TO_STONEPICKUP = new Pose2d(1.5, -54.2, Math.toRadians(-45));

    public static Pose2d GRAB_STONE = new Pose2d(-26.5, -49.25, Math.toRadians(-90));
    public static Pose2d GRAB_STONE2 = new Pose2d(-26.75, -48.6, Math.toRadians(-90));
    public static Pose2d GRAB_STONE3 = new Pose2d(-25.5, -50, Math.toRadians(-90));
    public static Pose2d GRAB_STONE4 = new Pose2d(-24.75, -50.5, Math.toRadians(-90));
    public static Pose2d GRAB_STONE5 = new Pose2d(-24.75, -51, Math.toRadians(-90));

    public static Pose2d GRAB_STONE6 = new Pose2d(-34.5, -47, Math.toRadians(-90));


    public static Pose2d DEPOSIT_TWO = new Pose2d(-0.6, -39, Math.toRadians(-90));
    public static Pose2d DEPOSIT_THREE = new Pose2d(-0.6, -38, Math.toRadians(-90));



    public static Pose2d CASE_3_MID = new Pose2d(27, -53, Math.toRadians(0));
    public static Pose2d CASE_1 = new Pose2d(-28.5, -50, Math.toRadians(-90));
    public static Pose2d CASE_3 = new Pose2d(18.7, -50, Math.toRadians(0));
    public static Pose2d CASE_2_TURN = new Pose2d(-3.2, -50, Math.toRadians(0));



/*
    public static Pose2d STOP = new Pose2d(1, -17, Math.toRadians(0));
    public static Pose2d FIRST_FINISH = new Pose2d(26, -16.8, Math.toRadians(0));
    public static Pose2d THIRD_FINISH = new Pose2d(-26, -16.6, Math.toRadians(0));
    public static Pose2d MID_GOAL_DROP = new Pose2d(-2.054, -19.1, Math.toRadians(48));
    public static Pose2d MID_GOAL_STOP = new Pose2d(0.736, -13.604, 0);
    public static Pose2d START = new Pose2d(0 , 0, 0);

    public static Pose2d FIRST_CONE_STOP = new Pose2d(-2.7, -36.3, Math.toRadians(44));
    public static Pose2d FIRST_CONE_DROP = new Pose2d(1.803, -31.155, Math.toRadians(90));
    public static Pose2d FIRST_CONE_TURN = new Pose2d(1.803, -31.155, Math.toRadians(90));
    public static Pose2d SECOND_CONE_GRAB = new Pose2d(1.799 , -31.148, Math.toRadians(90));
    public static Pose2d SECOND_CONE_RETURN = new Pose2d(-2.72, -36.33, Math.toRadians(44));
    public static Pose2d SECOND_CONE_DROP = new Pose2d(17.017, -31.065, Math.toRadians(90));
    public static Pose2d SECOND_CONE_TURN = new Pose2d(1.801, -31.150, Math.toRadians(44));
    public static Pose2d THIRD_CONE_GRAB = new Pose2d(1.800 , -31.150, Math.toRadians(90));
    public static Pose2d THIRD_CONE_RETURN = new Pose2d(-2.73, -36.34, Math.toRadians(44));
    public static Pose2d THIRD_CONE_DROP = new Pose2d(17.018, -31.063, Math.toRadians(90));
    public static Pose2d THIRD_CONE_TURN = new Pose2d(1.815, -31.157, Math.toRadians(90));
    public static Pose2d FOURTH_CONE_GRAB = new Pose2d(1.798, -31.149, Math.toRadians(44));
    public static Pose2d FOURTH_CONE_RETURN = new Pose2d(1.798 , -31.151, Math.toRadians(90));
    public static Pose2d FOURTH_CONE_DROP = new Pose2d(-2.74, -36.35, Math.toRadians(44));

 */




    public static Pose2d PARK_1 = new Pose2d(0,0,0);
    public static Pose2d PARK_2 = new Pose2d(0,0,0);
    public static Pose2d PARK_3 = new Pose2d(0,0,0);



    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();

        robot.localizer.reset();
        robot.setStartPose(new Pose2d(0, 3, 0));

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

            if (time.time() > 1 && time.time() < 5) {
                robot.drive.setPower(0,-0.2,0);
            } else {
                robot.drive.setPower(0,0,0);
            }

            if(time.time() > 8){
                if(coneCase == 0){
                    if(time.time() > 12 && time.time() < 17.5){
                        robot.drive.setPower(0.3,0,0);
                    }
                } else if (coneCase == 2){
                    if(time.time() > 12 && time.time() < 17.5){
                        robot.drive.setPower(-0.3,0,0);
                    }
                }
            }


                    /*
                case STRAFE:
                    points.add(new CurvePoint(CLEAR,1,1,4));
                    points.add(new CurvePoint(,0.6,1,4));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) >= Math.toRadians(1)) {
                        time.reset();
                        robot.arm.V4BOutPose();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(625);
                    } else {
                        if(time.time() > 0.7) {
                            robot.slides.setPosition(580, -0.3, 1);
                        } else {
                            robot.slides.setPosition(625);
                        }
                        if(time.time() > 0.9) {
                            robot.arm.GrabberPartial();
                        }

                        if(time.time() > 1.1){
                            robot.arm.GrabberClose();
                        }

                        if(time.time() > 1.2){
                            robot.arm.V4BFrontPose();
                        }

                        if(time.time() > 1.5) {
                            robot.arm.GrabberOpen();
                            if(robot.slides.isDown()){
                                robot.slides.reset();
                                robot.slides.setPower(0.0);
                            } else {
                                robot.slides.setPower(-0.2501);
                            }
                        }

                        if(time.time() > 1.7){
                            newState(State.TURN_AT_DEPOSIT_ONE);
                        }

                    }
                    break;

                case TURN_AT_DEPOSIT_ONE:
                    if(cycle == 0) {
                        points.add(new CurvePoint(STRAFE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE, 0.5, 1, 6));
                    } else if(cycle == 1){
                        points.add(new CurvePoint(MID_DEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(MID_CLEAR, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE2, 0.5, 1, 6));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE3, 0.75, 1, 6));
                    } else if (cycle == 3){
                        points.add(new CurvePoint( BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE4, 0.75, 1, 6));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE5, 0.75, 1, 6));
                    }else {
                        points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE6, 0.75, 1, 6));
                    }

                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.2501);
                    }

                    if(cycle==0){
                        robot.slides.setPosition(130, -0.2501, 1);
                    }

                    //ARM TUNING AREA

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 1.5) {
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
                        time.reset();
                    } else {
                        if(time.time() > 0.2) {
                            robot.arm.GrabberClose();
                        }
                        if (time.time() > 0.5) {
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
                        points.add(new CurvePoint(MID_DEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE2, 1.0, 1.0, 15));
                    } else if (cycle == 2){
                        points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE3, 1.0, 1.0, 15));
                    } else if (cycle == 3){
                        points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE4, 1.0, 1.0, 15));
                    } else if(cycle == 4){
                        points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE5, 1.0, 1.0, 15));
                    }else {
                        points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                        points.add(new CurvePoint(GRAB_STONE6, 1.0, 1.0, 15));
                    }

                    robot.arm.GrabberClose();
                    robot.arm.V4BOutPose();
                    if(time.time() > 0.2) {
                        newState(State.DEPOSIT_ONE);
                    }
                    break;

                //STACK TO DEPOSIT
                case DEPOSIT_ONE:
                    if(cycle == 0) {
                        points.add(new CurvePoint(GRAB_STONE, 1.0, 1.0, 15));
                        if(robot.getPos().vec().distTo(BACK_AT_DEPOSIT_ONE.vec()) > 15) {
                            points.add(new CurvePoint(BACK_AT_DEPOSIT_ONE, 1.0, 1.0, 6));
                            points.add(new CurvePoint(MID_DEPOSIT, 1.0, 1.0, 6));
                        }else{
                            points.add(new CurvePoint(BACK_AT_DEPOSIT_ONE, 1.0, 1.0, 15));
                            points.add(new CurvePoint(MID_DEPOSIT, 1.0, 1.0, 15));
                        }
                    } else if(cycle == 1){
                        points.add(new CurvePoint(GRAB_STONE, 1.0, 1.0, 15));
                        if(robot.getPos().vec().distTo(CURVE_FORBACKUPHIGH.vec()) > 15) {
                            points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 6));
                            points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 6));
                        }else{
                            points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 15));
                            points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        }
                    } else if (cycle == 2 || cycle == 3){
                        points.add(new CurvePoint(GRAB_STONE, 1.0, 1.0, 15));
                        if(robot.getPos().vec().distTo(CURVE_FORBACKUPHIGH.vec()) > 15) {
                            points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 6));
                            points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 6));
                        }else{
                            points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 15));
                            points.add(new CurvePoint(BACKUP_HIGHDEPOSIT, 1.0, 1.0, 15));
                        }
                    } else if(cycle == 4|| cycle == 5){
                        points.add(new CurvePoint(GRAB_STONE, 1.0, 1.0, 15));
                        if(robot.getPos().vec().distTo(CURVE_FORBACKUPHIGH.vec()) > 15) {
                            points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 6));
                            points.add(new CurvePoint(BACKUP_HIGHDEPOSIT_4_5, 1.0, 1.0, 6));
                        }else{
                            points.add(new CurvePoint(CURVE_FORBACKUPHIGH, 1.0, 1.0, 15));
                            points.add(new CurvePoint(BACKUP_HIGHDEPOSIT_4_5, 1.0, 1.0, 15));
                        }
                    }


                    if(cycle == 0) {
                        if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) >= Math.toRadians(1)) {
                            if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 25) {
                                robot.slides.setPosition(450);
                            }
                            time.reset();
                            robot.arm.GrabberClose();
                        } else {
                            if (time.time() > 0.7) {
                                robot.slides.setPosition(400, -0.3, 1);
                            } else {
                                robot.slides.setPosition(450);
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
                                if (robot.slides.isDown()) {
                                    robot.slides.reset();
                                    robot.slides.setPower(0.0);
                                } else {
                                    robot.slides.setPower(-0.2501);
                                }
                                cycle++;
                                if (cycle < 5) {
                                    newState(State.TURN_AT_DEPOSIT_ONE);
                                } else {
                                    if (coneCase == 0) {
                                        newState(State.CASE_0);
                                    } else if (coneCase == 1) {
                                        newState(State.CASE_1);
                                    } else {
                                        newState(State.CASE_2);
                                    }
                                }
                            }
                        }
                    } else if (cycle == 1 ||cycle == 2||cycle == 3||cycle == 4||cycle == 5){
                        if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) >= Math.toRadians(1)) {
                            if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 25) {
                                robot.slides.setPosition(640);
                            }
                            time.reset();
                            robot.arm.GrabberClose();
                        } else {
                            if (time.time() > 0.5) {
                                robot.slides.setPosition(580, -0.3, 1);
                            } else {
                                robot.slides.setPosition(640);
                            }
                            if (time.time() > 0.7) {
                                robot.arm.GrabberPartial();
                            }

                            if (time.time() > 0.9) {
                                robot.arm.GrabberClose();
                            }

                            if (time.time() > 1.0) {
                                robot.arm.V4BFrontPose();
                            }

                            if (time.time() > 1.2) {
                                if (robot.slides.isDown()) {
                                    robot.slides.reset();
                                    robot.slides.setPower(0.0);
                                } else {
                                    robot.slides.setPower(-0.2501);
                                }
                                cycle++;
                                if (cycle < 5) {
                                    newState(State.TURN_AT_DEPOSIT_ONE);
                                } else {
                                    if (coneCase == 0) {
                                        newState(State.CASE_0);
                                    } else if (coneCase == 1) {
                                        newState(State.CASE_1);
                                    } else {
                                        newState(State.CASE_2);
                                    }
                                }
                            }
                        }
                    }
                    break;

                case CASE_0:
                    points.add(new CurvePoint(BACKUP_HIGHDEPOSIT,1.0,1.0,15));
                    points.add(new CurvePoint(CURVE_FORBACKUPHIGH,1.0,1.0,15));
                    points.add(new CurvePoint(CASE_1,1.0,1.0,15));

                    robot.arm.V4BFrontHoldPos();
                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.2501);
                    }

                    break;

                case CASE_1:
                    points.add(new CurvePoint(STRAFE,1.0,1.0,15));
                    points.add(new CurvePoint(CASE_2_TURN,1.0,1.0,15));

                    robot.arm.V4BFrontHoldPos();
                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.2501);
                    }
                    break;

                case CASE_2:
                    points.add(new CurvePoint(STRAFE,1.0,1.0,10));
                    points.add(new CurvePoint(CASE_3,1.0,1.0,10));
                    robot.arm.V4BFrontHoldPos();
                    if(robot.slides.isDown()){
                        robot.slides.reset();
                        robot.slides.setPower(0.0);
                    } else {
                        robot.slides.setPower(-0.2501);
                    }
                    break;
                    /*
                case THIRD_STOP:
                    points.add(new CurvePoint(MID_GOAL_DROP,0.5,0.5,15));
                    points.add(new CurvePoint(STOP,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) <= Math.toRadians(2)) {
                        time.reset();
                    } else {
                        robot.arm.V4BFrontPose();
                        robot.arm.GrabberOpen();
                        if(time.time() > 0.5){
                            if(robot.slides.getPosition() > 60){
                                robot.slides.setPower(-0.5);
                            } else {
                                robot.slides.setPower(0.0);
                            }
                        }
                        if(time.time() > 2.2) {
                            newState(State.THIRD_FINISH);
                        }

                    }
                    break;

                case THIRD_FINISH:
                    points.add(new CurvePoint(STOP,0.5,0.5,15));
                    points.add(new CurvePoint(THIRD_FINISH,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                    } else {
                        if (time.time() > 2.2) {
                            newState(State.PARK);
                        }
                    }
                    break;

                case TRAVEL_TO_STONE_PICKUP:
                    points.add(new CurvePoint(DEPOSIT_ONE,0.5,0.5,15));
                    points.add(new CurvePoint(TRAVEL_TO_STONEPICKUP,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    } else {
                        if(time.time() > 0.75) {
                            robot.arm.V4BFrontPose();
                        }
                        if(time.time() > 1.5){
                            robot.arm.GrabberOpen();
                            if(robot.slides.getPosition() > 100){
                                robot.slides.setPower(-0.2501);
                            } else {
                                robot.slides.setPower(-0.2501);
                            }
                        }
                        if(time.time() > 2){
                            newState(State.GRAB_STONE);
                        }

                    }
                    break;

                case GRAB_STONE:
                    points.add(new CurvePoint(TRAVEL_TO_STONEPICKUP,0.5,0.5,15));
                    points.add(new CurvePoint(GRAB_STONE,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.V4BFrontPose();
                        robot.slides.setPosition(115); //90 for 2nd Cone //60 for 3rd Cone // 25 for 4th Cone
                        robot.arm.GrabberOpen();
                    } else {
                        robot.arm.GrabberClose();
                        if (time.time() > 0.7) {
                            robot.slides.setPosition(450);
                            newState(State.RETURN_TO_STONE_PICKUP);
                        }
                    }
                    break;

                case RETURN_TO_STONE_PICKUP:
                    points.add(new CurvePoint(GRAB_STONE,0.5,0.5,15));
                    points.add(new CurvePoint(DEPOSIT_ONE,0.5,0.5,15));
                    points.add(new CurvePoint(TRAVEL_TO_STONEPICKUP,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.V4BHoldPos();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    } else {
                        if(time.time() > 0.6) {
                            newState(State.PARK);
                        }
                    }
                    break;

                case TRAVEL_TO_DEPOSITTWO:
                    points.add(new CurvePoint(TRAVEL_TO_STONEPICKUP,0.5,0.5,15));
                    points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.V4BHoldPos();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    } else {
                        if(time.time() > 0.75) {
                            newState(State.DEPOSIT_TWO);
                        }
                    }
                    break;

                case DEPOSIT_TWO:
                    points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE,0.5,0.5,15));
                    points.add(new CurvePoint(DEPOSIT_TWO,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.arm.V4BHoldPos();
                        robot.slides.setPosition(450);
                    } else {
                        robot.slides.setPosition(450);
                        robot.arm.V4BOutPose();
                        if(time.time() > 0.75) {
                            robot.arm.GrabberOpen();
                        }

                        if(time.time() > 1.5){
                            robot.arm.GrabberClose();
                            newState(State.TRAVEL_TO_STONE_PICKUPTWO);
                        }
                    }
                    break;


                case TRAVEL_TO_STONE_PICKUPTWO:
                    points.add(new CurvePoint(DEPOSIT_TWO,0.5,0.5,15));
                    points.add(new CurvePoint(TRAVEL_TO_STONEPICKUP,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    } else {
                        if(time.time() > 0.75) {
                            robot.arm.V4BFrontPose();
                        }
                        if(time.time() > 1.5){
                            robot.arm.GrabberOpen();
                            if(robot.slides.getPosition() > 100){
                                robot.slides.setPower(-0.2501);
                            } else {
                                robot.slides.setPower(-0.2501);
                            }
                        }
                        if(time.time() > 2){
                            newState(State.GRAB_STONE_2);
                        }

                    }
                    break;

                case GRAB_STONE_2:
                    points.add(new CurvePoint(TRAVEL_TO_STONEPICKUP,0.5,0.5,15));
                    points.add(new CurvePoint(GRAB_STONE,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.V4BFrontPose();
                        robot.slides.setPosition(90); //90 for 2nd Cone //60 for 3rd Cone // 25 for 4th Cone
                        robot.arm.GrabberOpen();
                    } else {
                        robot.arm.GrabberClose();
                        if (time.time() > 0.75) {
                            robot.slides.setPosition(450);
                            newState(State.RETURN_TO_STONE_PICKUPTWO);
                        }
                    }
                    break;

                case RETURN_TO_STONE_PICKUPTWO:
                    points.add(new CurvePoint(GRAB_STONE,0.5,0.5,15));
                    points.add(new CurvePoint(TRAVEL_TO_STONEPICKUP,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 3.5) {
                        time.reset();
                        robot.arm.V4BHoldPos();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    } else {
                        if(time.time() > 0.75) {
                            newState(State.TRAVEL_TO_DEPOSITTHREE);
                        }
                    }
                    break;


                case TRAVEL_TO_DEPOSITTHREE:
                    points.add(new CurvePoint(TRAVEL_TO_STONEPICKUP,0.5,0.5,15));
                    points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE,0.5,0.5,15));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.V4BHoldPos();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    } else {
                        if(time.time() > 0.75) {
                            newState(State.DEPOSIT_THREE);
                        }
                    }
                    break;

                case DEPOSIT_THREE:
                    points.add(new CurvePoint(TURN_AT_DEPOSIT_ONE,0.5,0.5,15));
                    points.add(new CurvePoint(DEPOSIT_THREE,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.arm.V4BHoldPos();
                        robot.slides.setPosition(450);
                    } else {
                        robot.slides.setPosition(450);
                        robot.arm.V4BOutPose();
                        if(time.time() > 0.75) {
                            robot.arm.GrabberOpen();
                        }

                        if(time.time() > 1.25){
                            robot.arm.GrabberClose();
                            if(coneCase == 0){
                                newState(State.CASE_0);
                            } else if (coneCase == 1){
                                newState(State.CASE_1);
                            } else {
                                newState(State.CASE_2);
                            }
                        }
                    }
                    break;

                case CASE_0:
                    points.add(new CurvePoint(DEPOSIT_THREE,0.5,0.5,15));
                    points.add(new CurvePoint(STRAFE_TO_CASE,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    } else {
                        if(time.time() > 0.75) {
                            robot.arm.V4BFrontPose();
                        }
                        if(time.time() > 1.5){
                            robot.arm.GrabberOpen();
                            if(robot.slides.getPosition() > 100){
                                robot.slides.setPower(-0.2501);
                            } else {
                                robot.slides.setPower(-0.2501);
                            }
                        }
                        if(time.time() > 2){
                            newState(State.CASE_0_STRAFE);
                        }

                    }
                    break;

                case CASE_0_STRAFE:
                    points.add(new CurvePoint(STRAFE_TO_CASE,0.5,0.5,15));
                    points.add(new CurvePoint(CASE_1,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.arm.V4BHoldPos();
                    } else {
                        if(time.time() > 0.2) {
                            newState(State.CASE_0_TURN);
                        }

                    }
                    break;

                case CASE_0_TURN:
                    points.add(new CurvePoint(CASE_1,0.5,0.5,15));
                    points.add(new CurvePoint(CASE_1_TURN,0.5,0.5,15));

                    if(Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) >= Math.toRadians(2)) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.arm.V4BHoldPos();
                    } else {
                        newState(State.PARK);
                    }
                    break;

                case CASE_1:
                    points.add(new CurvePoint(DEPOSIT_THREE,0.5,0.5,15));
                    points.add(new CurvePoint(CASE_2_TURN,0.5,0.5,15));

                    if(Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) >= Math.toRadians(3)) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    }  else {
                        if(time.time() > 0.75) {
                            robot.arm.V4BFrontPose();
                        }
                        if(time.time() > 1.5){
                            robot.arm.GrabberOpen();
                            if(robot.slides.getPosition() > 100){
                                robot.slides.setPower(-0.2501);
                            } else {
                                robot.slides.setPower(-0.2501);
                            }
                        }
                        if(time.time() > 2){
                            newState(State.PARK);
                        }
                    }
                    break;

                case CASE_2:
                    points.add(new CurvePoint(DEPOSIT_THREE,0.5,0.5,15));
                    points.add(new CurvePoint(STRAFE_TO_CASE,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.slides.setPosition(450);
                    } else {
                        if(time.time() > 0.75) {
                            robot.arm.V4BFrontPose();
                        }
                        if(time.time() > 1.5){
                            robot.arm.GrabberOpen();
                            if(robot.slides.getPosition() > 100){
                                robot.slides.setPower(-0.2501);
                            } else {
                                robot.slides.setPower(-0.2501);
                            }
                        }
                        if(time.time() > 2){
                            newState(State.CASE_2_STRAFE);
                        }

                    }
                    break;
                case CASE_2_STRAFE:
                    points.add(new CurvePoint(STRAFE_TO_CASE,0.5,0.5,15));
                    points.add(new CurvePoint(CASE_3,0.5,0.5,15));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.arm.V4BHoldPos();
                    } else {
                        if(time.time() > 0.2) {
                            newState(State.CASE_2_TURN);
                        }

                    }
                    break;
                case CASE_2_TURN:
                    points.add(new CurvePoint(CASE_3,0.5,0.5,15));
                    points.add(new CurvePoint(CASE_3_TURN,0.5,0.5,15));

                    if(Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) >= Math.toRadians(2)) {
                        time.reset();
                        robot.arm.GrabberClose();
                        robot.arm.V4BHoldPos();
                    } else {
                        newState(State.PARK);
                    }
                    break;
                    /*
                case THIRD_STOP:
                    points.add(new CurvePoint(MID_GOAL_DROP,0.5,0.5,15));
                    points.add(new CurvePoint(STOP,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) <= Math.toRadians(2)) {
                        time.reset();
                    } else {
                        robot.arm.V4BFrontPose();
                        robot.arm.GrabberOpen();
                        if(time.time() > 0.5){
                            if(robot.slides.getPosition() > 60){
                                robot.slides.setPower(-0.5);
                            } else {
                                robot.slides.setPower(0.0);
                            }
                        }
                        if(time.time() > 2.2) {
                            newState(State.THIRD_FINISH);
                        }

                    }
                    break;

                case THIRD_FINISH:
                    points.add(new CurvePoint(STOP,0.5,0.5,15));
                    points.add(new CurvePoint(THIRD_FINISH,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                    } else {
                        if (time.time() > 2.2) {
                            newState(State.PARK);
                        }
                    }
                    break;
                case SECOND_STOP:
                    points.add(new CurvePoint(MID_GOAL_DROP,0.5,0.5,15));
                    points.add(new CurvePoint(STOP,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) <= Math.toRadians(2)) {
                        time.reset();
                    } else {
                        robot.arm.V4BFrontPose();
                        robot.arm.GrabberOpen();
                        if(time.time() > 0.5){
                            if(robot.slides.getPosition() > 60){
                                robot.slides.setPower(-0.5);
                            } else {
                                robot.slides.setPower(0.0);
                            }
                        }
                        if(time.time() > 2.2) {
                            newState(State.PARK);
                        }

                    }
                    break;

                case FIRST_STOP:
                    points.add(new CurvePoint(MID_GOAL_DROP,0.5,0.5,15));
                    points.add(new CurvePoint(STOP,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) <= Math.toRadians(2)) {
                        time.reset();
                    } else {
                        robot.arm.V4BFrontPose();
                        robot.arm.GrabberOpen();
                        if(time.time() > 0.5){
                            if(robot.slides.getPosition() > 60){
                                robot.slides.setPower(-0.5);
                            } else {
                                robot.slides.setPower(0.0);
                            }
                        }
                        if(time.time() > 2.2) {
                            newState(State.FIRST_FINISH);
                        }

                    }
                    break;
                case FIRST_FINISH:
                    points.add(new CurvePoint(STOP,0.5,0.5,15));
                    points.add(new CurvePoint(FIRST_FINISH,0.5,0.5,15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2) {
                        time.reset();
                    } else {
                        if (time.time() > 2.2) {
                            newState(State.PARK);
                        }
                    }
                    break;

                    /*
               // case FOURTH_CONE_DROP:
                   // points.add(new CurvePoint(FOURTH_CONE_RETURN, 0.5, 0.5, 15));
                    //points.add(new CurvePoint(FOURTH_CONE_DROP, 0.5, 0.5, 15));
                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) > 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) <= Math.toRadians(2)) {
                      //  time.reset();
                        //robot.arm.GrabberClose();
                        //robot.arm.V4BHighOutPose();
                    } else {
                        robot.arm.GrabberOpen();
                        if(time.time() > 1) {
                            newState(State.PARK);
                        }

                    }
                    break;
                    */

            robot.drive.write();
            robot.arm.write();
            robot.slides.write();

            telemetry.update();
        }
    }
}




    /*
    if(cycle == 0) {
                            //robot.slides.setPosition(secondStackPos);
                            robot.arm.GrabberOpen();
                            robot.arm.V4BFrontPose();
                        } else if (cycle == 1){
                            //robot.slides.setPosition(secondStackPos);
                            robot.arm.GrabberOpen();
                            robot.arm.V4BFrontPose();
                        } else if (cycle == 2){
                            //robot.slides.setPosition(secondStackPos);
                            robot.arm.GrabberOpen();
                            robot.arm.V4BFrontPose();
                        } else {
                            newState(State.PARK);
                        }
     */
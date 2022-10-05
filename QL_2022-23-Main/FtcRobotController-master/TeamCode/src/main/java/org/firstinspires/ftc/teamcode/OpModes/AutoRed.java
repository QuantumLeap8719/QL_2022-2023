package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class AutoRed extends LinearOpMode {

    private enum State{
        DROP_INTAKE,
        PRE_LOAD_FREIGHT,
        CAROUSEL,
        RUN_CAROUSEL,
        DEPOT,
        CYCLE,
        CYCLE2,
        PARK
    }
    State mRobotState = State.DROP_INTAKE;

    boolean gtp = false;
    int cycle = 0;
    int numCycles = 2;
    ElapsedTime time;
    Robot robot;
    public static Pose2d PRE_LOADED_FREIGHT_POS = new Pose2d(22.156,20.433, Math.toRadians(180));
    public static Pose2d FREIGHT_POS = new Pose2d(15.044,25.596, Math.toRadians(205.460));
    public static Pose2d FREIGHT_POS_2 = new Pose2d(13.947,22.032, Math.toRadians(200));
    public static Pose2d FREIGHT_POS_3 = new Pose2d(4.588,28.317, Math.toRadians(230.5));
    public static Pose2d CAROUSEL_POS = new Pose2d(70,13, Math.toRadians(180));
    public static Pose2d DEPOT_POS = new Pose2d(-35,0.5, Math.toRadians(270));
    public static Pose2d PARK_POS = new Pose2d(-30,0.5, Math.toRadians(270));

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();

        robot.arm.closeFront();
        robot.arm.front.write();
        telemetry.update();
        robot.intake.clamp();
        robot.intake.write();

        robot.localizer.reset();
        robot.setStartPose(new Pose2d(0, 0,Math.toRadians(90)));

        waitForStart();

        time.startTime();

        while(opModeIsActive()){
            ArrayList<CurvePoint> points = new ArrayList<>();

            switch(mRobotState) {
                case DROP_INTAKE:
                    if(time.time() >= 0.2){
                        robot.localizer.reset();
                        newState(State.PRE_LOAD_FREIGHT);
                    }

                    robot.intake.drop();
                    break;
                case PRE_LOAD_FREIGHT:
                    points.add(new CurvePoint(new Pose2d(0, 0, Math.toRadians(270)), 1d, 1d, 25));
                    points.add(new CurvePoint(PRE_LOADED_FREIGHT_POS, 1d, 1d, 25));

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.5) {
                        if (time.time() > 0.25) {
                            time.reset();
                            newState(State.CAROUSEL);
                        }

                        robot.arm.release();
                    } else {
                        robot.arm.V4BPartialOutPose();
                        time.reset();
                    }
                    break;

                case CAROUSEL:
                    points.add(new CurvePoint(PRE_LOADED_FREIGHT_POS, 1d, 1d, 15));
                    points.add(new CurvePoint(new Pose2d(37.3, PRE_LOADED_FREIGHT_POS.getY() + 0.1, Math.PI), 1d, 1d, 15));
                    points.add(new CurvePoint(CAROUSEL_POS, 1d, 1d, 15));

                    if(robot.getPos().vec().distTo(PRE_LOADED_FREIGHT_POS.vec()) > 15){
                        robot.arm.close();
                        robot.arm.reset();
                        robot.arm.openFront();
                    }

                    if( robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2) {
                        gtp = true;
                        time.reset();
                        newState(State.RUN_CAROUSEL);
                    }
                    break;
                case RUN_CAROUSEL:
                    points.add(new CurvePoint(new Pose2d(69.41, 10.5, Math.PI), 0.5d, 1d, 25));
                    if(time.time() > 3.5){
                        gtp = false;
                        robot.carousel.stopCarousel();
                        newState(State.DEPOT);
                    }else{
                        robot.carousel.runCarousel();
                    }
                    break;
                case DEPOT:
                    points.add(new CurvePoint(CAROUSEL_POS, 1d, 1d, 15));
                    points.add(new CurvePoint(new Pose2d(20, DEPOT_POS.getY() - 0.1, Math.toRadians(270)), 1d,1d,15));
                    points.add(new CurvePoint(DEPOT_POS, 1.0, 1d, 15));

                    if(robot.getPos().getX() < 0){
                        robot.intake.intake(true);
                    }

                    if( Math.abs(robot.getPos().getX() - DEPOT_POS.getX()) < 1.0) {
                        time.reset();
                        newState(State.CYCLE);
                    } else {
                        time.reset();
                    }
                    break;

                case CYCLE:
                    robot.slides.setBrake();
                    points.add(new CurvePoint(DEPOT_POS, 1d, 1d, 15));
                    points.add(new CurvePoint(new Pose2d(5, 0, Math.toRadians(270)), 1d, 1d, 15)); //2nd POINT FOR FREIGHT_CYCLE
                    /*switch(cycle){
                        case 0:
                        case 3:
                            points.add(new CurvePoint(FREIGHT_POS, 1d, 1d, 15));
                            break;
                        case 1:
                            points.add(new CurvePoint(FREIGHT_POS_2, 1d, 1d, 15));
                            break;
                        case 2:
                            points.add(new CurvePoint(FREIGHT_POS_3, 1d, 1d, 15));
                            break;
                    }*/
                    points.add(new CurvePoint(FREIGHT_POS, 0.5d, 0.5d, 15));

                    if(robot.getPos().getX() > -5) {
                        robot.arm.closeFront();
                        robot.slides.setPosition(157);
                        robot.arm.V4BPartialOutPose();
                        robot.intake.stop();
                    }else if(robot.getPos().getX() <= 0){
                        if(time.time() > 0.5) {
                            robot.intake.intake(false);
                        }
                    }

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < 2.0) {
                        if(time.time() > 0.2) {
                            newState(State.CYCLE2);
                        }
                        robot.arm.release();
                    }else{
                        if(robot.getPos().getX() > 0) {
                            time.reset();
                        }
                    }
                    break;

                case CYCLE2:
                    points.add(new CurvePoint(FREIGHT_POS, 1d, 1d, 10));
                    points.add(new CurvePoint(new Pose2d(5, -1, Math.toRadians(270)), 1d, 1d, 10)); //2nd POINT FOR FREIGHT_CYCLE
                    if(cycle <= numCycles) {
                        points.add(new CurvePoint(new Pose2d(DEPOT_POS.getX() - cycle * 4, DEPOT_POS.getY(), DEPOT_POS.getHeading()), 1d, 1d, 10));
                    }else{
                        points.add(new CurvePoint(PARK_POS, 1d, 1d, 10));
                    }

                    if(robot.getPos().getX() < 0) {
                        robot.intake.intake(true);
                    }

                    robot.arm.close();
                    robot.arm.reset();
                    robot.arm.openFront();

                    robot.slides.setCoast();
                    if(robot.slides.getPosition() < 50){
                        if(!robot.slides.isDown()){
                            telemetry.addLine("not down");
                            robot.slides.setPower(-0.2);
                        }else{
                            telemetry.addLine("down");
                            robot.slides.setPower(-0.03);
                        }
                    }else{
                        robot.slides.setPower(-robot.slides.downPower);
                    }

                    if (robot.getPos().getX() - 1 < points.get(points.size() - 1).x) {
                        if(cycle <= numCycles) {
                            cycle++;
                            newState(State.CYCLE);
                        } else {
                            newState(State.PARK);
                        }
                    }else {
                        time.reset();
                    }

                    break;
                case PARK:
                    gtp = true;
                    points.add(new CurvePoint(PARK_POS, 1d, 1d, 10));

                    if(time.time() < 1){
                        robot.intake.intake(true);
                    } else {
                        robot.intake.stop();
                    }

                    break;
            }

            if(points.size() != 0){
                if(!gtp){
                    RobotMovement.followCurve(points, robot, telemetry);
                }else{
                    robot.GoTo(points.get(points.size() - 1).toPose(),new Pose2d(points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).turnSpeed));
                }
            }else{
                robot.drive.setPower(0, 0, 0);
                robot.drive.write();
                robot.updatePos();
            }

            telemetry.addData("State", mRobotState);
            telemetry.addData("Position", robot.getPos());
            telemetry.update();
            robot.arm.write();
            robot.slides.write();
            robot.carousel.write();
            robot.intake.write();
        }
    }

    public void newState(State state){
        time.reset();
        mRobotState = state;
    }
}

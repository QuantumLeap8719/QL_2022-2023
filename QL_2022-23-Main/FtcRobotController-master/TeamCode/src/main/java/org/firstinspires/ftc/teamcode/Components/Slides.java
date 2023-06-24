package org.firstinspires.ftc.teamcode.Components;

import android.service.vr.VrListenerService;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@Config
public class Slides {
    public Caching_Motor lSlide;
    public Caching_Motor rSlide;

    public static double kp = 0.03;//0.02;
    public static double ki = 0.0;
    public static double kd = 0.0004;//0.0008;
    public static double gff = 0.25;//0.25;

    public static double high_goal_position = 530;//326;
    public static double mid_goal_position = 340;
    public static double low_goal_position = 130;
    public static double downPower = -0.2501;//0.245;

    public static int goalToggle = 0;
    boolean low;
    boolean mid;
    boolean high;

    private int bumper = 0;

    public PIDFController controller;
    Telemetry telemetry;
    public STATE mRobotState;
    DigitalChannel digitalTouch;
    ElapsedTime time;
    double startTime = 0;

    public enum STATE{
        AUTOMATION,
        MANUAL,
        DEPOSIT,
        DOWN,
        IDLE
    }

    public Slides(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        rSlide = new Caching_Motor(map, "rslide");
        lSlide = new Caching_Motor(map, "lslide");
        controller = new PIDFController(new PIDCoefficients(kp, ki, kd));
        reset();
        mRobotState = STATE.DOWN;
        time = new ElapsedTime();
        time.startTime();
        goalToggle = 2;
        digitalTouch = map.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        setBrake();
    }

    public void write(){
        rSlide.write();
        lSlide.write();
    }

    public void setPosition(double target){
        controller.setTargetPosition(target);
        setPower(controller.update(getPosition()));

        telemetry.addData("Target", target);
        telemetry.addData("Error", controller.getLastError());
    }

    public void setPosition(double target, double lowerBound, double upperBound){
        controller.setTargetPosition(target);
        setPower(Range.clip(controller.update(getPosition()), lowerBound, upperBound));

        telemetry.addData("Target", target);
        telemetry.addData("Error", controller.getLastError());
    }

    public void reset(){
        lSlide.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rSlide.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPosition(){
        double slidePos = (rSlide.motor.getCurrentPosition() - lSlide.motor.getCurrentPosition())/2.0;
        telemetry.addData("Slide Position", slidePos);
        return Math.abs(slidePos);
    }

    public void setCoast(){
        rSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrake(){
        rSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power){
        if(getPosition() > 10) {
            power += gff;
        }

        rSlide.setPower(power);
        lSlide.setPower(-power);
    }

    public void setPowerTele(double power){
        if(getPosition() > 10 && (mRobotState == STATE.MANUAL || mRobotState == STATE.AUTOMATION)) {
            power += gff;
        }

        rSlide.setPower(power);
        lSlide.setPower(-power);
    }

    public boolean isDown(){
        return !digitalTouch.getState();
    }

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2){
        switch (mRobotState) {
            case MANUAL:
                if (gamepad2.gamepad.left_stick_y > 0.1) {
                    setPowerTele(gamepad2.gamepad.left_stick_y * 0.5);
                } else if (gamepad2.gamepad.left_stick_y < -0.1) {
                    setPowerTele(gamepad2.gamepad.left_stick_y * 0.5);
                } else {
                    if (getPosition() > 500) {
                        setPowerTele(0.05);
                    } else {
                        setPowerTele(0);
                    }
                }
/*
                if(gamepad2.isPress(GamepadEx.Control.right_trigger)){
                    if(V4B_Arm.armToggle) {
                        mRobotState = STATE.AUTOMATION;
                    }else{
                        mRobotState = STATE.DOWN;
                    }
                }

 */
                break;
            case DEPOSIT:
                if (goalToggle == 0) {
                    setPosition(low_goal_position - 50, -0.3, 1);
                } else if (goalToggle == 1) {
                    setPosition(mid_goal_position - 50, -0.3, 1);
                } else if (goalToggle == 2) {
                    setPosition(high_goal_position - 50, -0.3, 1);
                }
                break;
            case AUTOMATION:
                if (gamepad2.gamepad.left_stick_y > 0.1 || gamepad2.gamepad.left_stick_y < -0.1) {
                    mRobotState = STATE.MANUAL;
                }

                if (goalToggle == 0) {
                    setPosition(low_goal_position);
                } else if (goalToggle == 1) {
                    setPosition(mid_goal_position);
                } else if (goalToggle == 2) {
                    setPosition(high_goal_position);
                }

                if (gamepad2.isPress(GamepadEx.Control.right_trigger)) {
                    mRobotState = STATE.DOWN;
                }
                break;
            case DOWN:
                if (isDown()) {
                    reset();
                    setPowerTele(0.0);
                } else {
                    setPowerTele(downPower);
                }
                if (gamepad2.gamepad.left_stick_y > 0.1 || gamepad2.gamepad.left_stick_y < -0.1) {
                    mRobotState = STATE.MANUAL;
                }

                if (V4B_Arm.grabberToggle == 2) {
                    mRobotState = STATE.AUTOMATION;
                }

                if (V4B_Arm.stackCase == 2) {
                    mRobotState = STATE.AUTOMATION;
                }

                break;
        }

        if(gamepad1.isPress(GamepadEx.Control.right_bumper)){
            time.reset();
        }

        if(V4B_Arm.grabberToggle == 0 || V4B_Arm.grabberToggle == 4){
            mRobotState = STATE.DOWN;
        }

        if(V4B_Arm.grabberToggle == 3){
                mRobotState = STATE.DEPOSIT;
        }

        if(V4B_Arm.stackCase == 3){
            mRobotState = STATE.DEPOSIT;
        }

        if(V4B_Arm.groundCase == 3){
            setPosition(30);
        }

        if(V4B_Arm.groundCase == 4){
            mRobotState = STATE.DOWN;
        }

        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 5 && V4B_Arm.stackCase == 1){
            setPosition(90);
        }
        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 5 && V4B_Arm.stackCase == 0){
            setPosition(90);
        }

        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 4 && V4B_Arm.stackCase == 1){
            setPosition(55);
        }

        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 4 && V4B_Arm.stackCase == 0){
            setPosition(55);
        }

        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 3 && V4B_Arm.stackCase == 1){
            setPosition(18);
        }
        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 3 && V4B_Arm.stackCase == 0){
            setPosition(18);
        }

        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 2 && V4B_Arm.stackCase == 1){
            mRobotState = STATE.DOWN;
        }
        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 2 && V4B_Arm.stackCase == 0){
            mRobotState = STATE.DOWN;
        }

        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 1 && V4B_Arm.stackCase == 1){
            mRobotState = STATE.DOWN;
        }
        if(V4B_Arm.slideToggle && V4B_Arm.stackToggle == 1 && V4B_Arm.stackCase == 0){
            mRobotState = STATE.DOWN;
        }




        if(V4B_Arm.stackCase == 3){
            if(gamepad1.gamepad.left_stick_y > 0.1 || gamepad1.gamepad.right_stick_x > 0.1 || gamepad1.gamepad.left_stick_x > 0.1){
                mRobotState = STATE.DOWN;
            }
        }

        if(gamepad1.isPress(GamepadEx.Control.y) || gamepad2.isPress(GamepadEx.Control.y) /*&& mRobotState == STATE.DOWN*/){
            goalToggle = 2;
        }

        if(gamepad1.isPress(GamepadEx.Control.b) || gamepad2.isPress(GamepadEx.Control.b) /*&& mRobotState == STATE.DOWN*/){
            goalToggle = 1;
        }

        if(gamepad1.isPress(GamepadEx.Control.a) || gamepad2.isPress(GamepadEx.Control.a)/*&& mRobotState == STATE.DOWN*/){
            goalToggle = 0;
        }

        if(gamepad1.isPress(GamepadEx.Control.x)){
            mRobotState = STATE.DOWN;
        }

        //Top Cone: 116
        //Cone 2:89
        //Cone 3:68
        //Cone 4:41
        //Cone 5:0

        telemetry.addData("Goal Toggle: ", goalToggle);
        telemetry.addData("State: ", mRobotState);

        telemetry.addData("isDown?", isDown());

        telemetry.addData("Slide Position: ", getPosition());
        telemetry.addData("Left", lSlide.motor.getCurrentPosition());
        telemetry.addData("Right", rSlide.motor.getCurrentPosition());
        telemetry.addData("Right Slide Power: ", rSlide.motor.getPower());
        telemetry.addData("Left Slide Power: ", lSlide.motor.getPower());
        telemetry.addData("Left Stick", gamepad2.gamepad.left_stick_y);


    }
}

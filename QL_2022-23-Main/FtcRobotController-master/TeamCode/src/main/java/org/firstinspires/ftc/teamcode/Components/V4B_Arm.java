package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class V4B_Arm {
    Caching_Servo rightArm;
    Caching_Servo leftArm;
    Caching_Servo grabber;

    private double rightArmPosition;
    private double leftArmPosition;
    ElapsedTime time = new ElapsedTime();
    ElapsedTime secondTime = new ElapsedTime();

    private double leftGrab = 0.9;
    private double rightGrab = 0.1;

    private double leftHoldPos = 0.72;
    private double rightHoldPos = 0.315;

    private double leftOutMid = 0.083;
    private double rightOutMid = 0.95;

    private double leftFrontPos = 0.997;
    private double rightFrontPos = 0.05;
    private double grabberOpen = 0.15;
    private double grabberPartialOpen = 0.25;
    private double grabberClose = 0.45;

    public static boolean armToggle = false;
    public static int grabberToggle = 0;
    private int dropToggle = 0;


    public V4B_Arm(HardwareMap map){
        rightArm = new Caching_Servo(map, "rightarm");
        leftArm = new Caching_Servo(map, "leftarm");
        grabber = new Caching_Servo(map,"grabber");
        rightArm.setPosition(rightFrontPos);
        leftArm.setPosition(leftFrontPos);
        grabber.setPosition(grabberClose);
        armToggle = false;
        rightArm.write();
        leftArm.write();
        grabberToggle = 0;
    }

    public void start(){
        time.startTime();
    }

    public void secondStart() {
        secondTime.startTime();
    }

    public void reset(){
        time.reset();
        secondTime.reset();
    }


    public void V4BGrab(){
        leftArm.setPosition(leftGrab);
        rightArm.setPosition(rightGrab);
    }

    public void V4BFrontPose(){
        leftArm.setPosition(leftFrontPos);
        rightArm.setPosition(rightFrontPos);
    }

    public void V4BOutPose(){
        leftArm.setPosition(leftOutMid);
        rightArm.setPosition(rightOutMid);
    }

    public void V4BHoldPos(){
        leftArm.setPosition(leftHoldPos);
        rightArm.setPosition(rightHoldPos);
    }

    public void setPosition(double position){
        leftArm.setPosition(position);
    }

    public void manualSetPosition(double left, double right){
        leftArm.setPosition(left);
        rightArm.setPosition(right);
    }

    public void GrabberOpen(){
        grabber.setPosition(grabberOpen);
    }

    public void GrabberClose(){
        grabber.setPosition(grabberClose);
    }

    public void GrabberPartial(){grabber.setPosition(grabberPartialOpen);}

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry) {
        if(gamepad2.isPress(GamepadEx.Control.right_bumper)) {
                leftArm.setPosition(leftOutMid);
                rightArm.setPosition(rightOutMid);
        }

        /*
        if(armToggle) {
            if (time.time() > 0.4) {
                rightArm.setPosition(rightArmPosition);
                leftArm.setPosition(leftArmPosition);
            } else {
                GrabberClose();
            }
        }else{
                V4BFrontPose();
                GrabberOpen();
        }


 */


        if(gamepad2.isPress(GamepadEx.Control.right_bumper)){
            grabberToggle = 2;
        }

        if(gamepad.isPress(GamepadEx.Control.right_bumper)){
            grabberToggle += 1;
            time.reset();
        }

        if(gamepad.isPress(GamepadEx.Control.x)){
            time.reset();
            grabberToggle = 5;
        }
/*
        if (gamepad.isPress(GamepadEx.Control.right_bumper)) {
            if (position < 1) {
               position += 0.05;
            }
        } else if (gamepad.isPress(GamepadEx.Control.left_bumper)) {
            if (position > 0) {
                position -= 0.05;
            }
        }


 */

        telemetry.addData("Drop", dropToggle);
        telemetry.addData("LeftArm", leftArm.getPosition());
        telemetry.addData("RightArm", rightArm.getPosition());

        if(gamepad.isPress(GamepadEx.Control.left_bumper) && grabberToggle == 1){
            leftArm.setPosition(leftHoldPos);
            rightArm.setPosition(rightHoldPos);
        }

        if(grabberToggle == 1){
            GrabberClose();
        } else if(grabberToggle == 2){
                leftArm.setPosition(leftOutMid);
                rightArm.setPosition(rightOutMid);
            GrabberClose();
        } else if(grabberToggle == 3){
            GrabberClose();
        }
        else if(grabberToggle == 4) {
            GrabberPartial();
            if(time.time() > 0.2){
                GrabberClose();
            }
            if(time.time() > 0.3){
                leftArm.setPosition(leftFrontPos);
                rightArm.setPosition(rightFrontPos);
            }
            if(time.time() > 1){
                grabberToggle = 0;
            }
        }
        else if(grabberToggle == 5){
            leftArm.setPosition(leftFrontPos);
            rightArm.setPosition(rightFrontPos);
            if(time.time() > 0.5){
                GrabberOpen();
                grabberToggle = 0;
            }
        } else {
            GrabberOpen();
        }


        //dtelemetry.addData("Position", position);
/*
        if(gamepad.isPress(GamepadEx.Control.x)){
            grabberPosToggle += 1;
        }

        if(grabberPosToggle == 1){
            GrabberClose();
        } else {
            grabberPosToggle = 0;
            GrabberOpen();
        }

 */
    }
    public void write(){
        rightArm.write();
        leftArm.write();
        grabber.write();
    }
}

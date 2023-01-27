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


    private double front_hold = 0.37;
    private double hold = 0.7;

    private double out = 1;
    private double front = 0.02;
    private double grabberOpen = 0.35;
    private double grabberPartialOpen = 0.35;
    private double grabberClose = 0.55;

    public static boolean armToggle = false;
    public static int grabberToggle = 0;
    private int dropToggle = 0;


    public V4B_Arm(HardwareMap map){
        rightArm = new Caching_Servo(map, "rightarm");
        leftArm = new Caching_Servo(map, "leftarm");
        grabber = new Caching_Servo(map,"grabber");
        leftArm.setZeros(.01, .93);
        rightArm.setZeros(.01, .94);
        manualSetPosition(front_hold);
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

    public void grabberPos(double pos){
        grabber.setPosition(pos);
    }

    public void manualSetPosition(double val){
        leftArm.setPosition(1 - val);
        rightArm.setPosition(val);
    }

    public void V4BHoldPos(){
        manualSetPosition(hold);
    }
    public void V4BFrontPose(){
        manualSetPosition(front);
    }

    public void V4BOutPose(){
        manualSetPosition(out);
    }

    public void GrabberOpen(){
        grabber.setPosition(grabberOpen);
    }

    public void GrabberClose(){
        grabber.setPosition(grabberClose);
    }

    public void GrabberAutoClose(){
        grabber.setPosition(0.55);
    }

    public void GrabberPartial(){grabber.setPosition(grabberPartialOpen);}

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry) {

        if(gamepad.isPress(GamepadEx.Control.right_bumper)){
            grabberToggle += 1;
            time.reset();
        }

        if(gamepad.isPress(GamepadEx.Control.x)){
            time.reset();
            grabberToggle = 1;
        }
/*
        telemetry.addData("Drop", dropToggle);
        telemetry.addData("LeftArm", leftArm.getPosition());
        telemetry.addData("RightArm", rightArm.getPosition());

 */
        telemetry.addData("GrabberToggle", grabberToggle);



        if(gamepad.isPress(GamepadEx.Control.left_bumper) && grabberToggle == 2){
            manualSetPosition(hold);
        }

        if(grabberToggle == 1){
            GrabberOpen();
        } else if(grabberToggle == 2){
            GrabberClose();

        } else if(grabberToggle == 3){
            manualSetPosition(out);
            GrabberClose();
        }
        else if(grabberToggle == 4) {
            GrabberClose();

        }
        else if(grabberToggle == 5){
            GrabberOpen();
            if(time.time() > 0.2){
                GrabberClose();
            }
            if(time.time() > 0.3){
                manualSetPosition(front_hold);
            }
        } else{
            manualSetPosition(front);
            if(time.time() > 0.25){
                grabberToggle = 1;
            }
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

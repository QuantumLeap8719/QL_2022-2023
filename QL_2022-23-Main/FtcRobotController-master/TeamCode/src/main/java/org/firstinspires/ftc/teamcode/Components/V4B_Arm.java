package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class V4B_Arm {
    public enum ARM_STATE{
        STACK,
        NORMAL
    }

    public ARM_STATE mRobotState;
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
    private double front = 0.0;
    private double terminal = 0.04;
    private double grabberOpen = 0.6;
    private double grabberPartialOpen = 0.6;
    private double grabberClose = 0.77;

    private double stack_five;
    private double stack_four;
    private double stack_three;
    private double stack_two;
    private double stack_one;


    public static boolean armToggle = false;
    public static int grabberToggle = 5;
    public static int stackToggle = 5;
    public static int stackCase = 0;
    private int dropToggle = 0;


    public V4B_Arm(HardwareMap map){
        mRobotState = ARM_STATE.NORMAL;
        rightArm = new Caching_Servo(map, "rightarm");
        leftArm = new Caching_Servo(map, "leftarm");
        grabber = new Caching_Servo(map,"grabber");
        leftArm.setZeros(.01, .93);
        rightArm.setZeros(.01, .92);
        grabberToggle = 5;
        stackToggle = 5;
        manualSetPosition(front_hold);
        grabber.setPosition(grabberClose);
        armToggle = false;
        rightArm.write();
        leftArm.write();
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

    public void V4BFrontHoldPos(){
        manualSetPosition(front_hold);
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
        grabber.setPosition(0.42);
    }

    public void GrabberPartial(){grabber.setPosition(grabberPartialOpen);}

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry) {

        switch(mRobotState){
            case NORMAL:
                if(gamepad.isPress(GamepadEx.Control.right_bumper)){
                    grabberToggle += 1;
                    time.reset();
                }

                if(gamepad.isPress(GamepadEx.Control.x)){
                    grabberToggle = 7;
                    time.reset();
                }

                if(gamepad.isPress(GamepadEx.Control.left_bumper)){
                    time.reset();
                    grabberToggle = 0;
                }

                if(grabberToggle == 1){
                    GrabberClose();
                    if(time.time() > 0.2){
                        manualSetPosition(hold);
                    }
                } else if(grabberToggle == 2){
                    manualSetPosition(out);

                } else if(grabberToggle == 3){
                    if(time.time() > 0.2){
                        GrabberOpen();
                        if(time.time() > 0.39){
                            GrabberClose();
                        }
                        if(time.time() > 0.45){
                            manualSetPosition(front_hold);
                        }
                    }
                } else if (grabberToggle == 5){
                    manualSetPosition(front_hold);
                    grabber.setPosition(grabberClose);
                }
                else if (grabberToggle == 7){
                    manualSetPosition(terminal);
                }
                else{
                    grabberToggle = 0;
                    manualSetPosition(front);
                    if(time.time() > 0.2){
                        GrabberOpen();
                    }
                }

                if(gamepad2.isPress(GamepadEx.Control.right_bumper)){
                    grabberToggle = 0;
                    mRobotState = ARM_STATE.STACK;
                }
                break;





            case STACK:

                if(gamepad.isPress(GamepadEx.Control.right_bumper)){
                    stackCase += 1;
                    time.reset();
                }

                if(gamepad.isPress(GamepadEx.Control.x)){
                    stackCase = 7;
                    time.reset();
                }

                if(gamepad.isPress(GamepadEx.Control.left_bumper)){
                    time.reset();
                    stackCase = 0;
                }

            if(stackCase == 1){
                    GrabberClose();
                    if(time.time() > 0.2){
                        manualSetPosition(hold);
                    }
                } else if(stackCase == 2){
                    manualSetPosition(out);

                } else if(stackCase == 3){
                    if(time.time() > 0.2){
                        GrabberOpen();
                        if(time.time() > 0.39){
                            GrabberClose();
                        }
                        if(time.time() > 0.45){
                            manualSetPosition(front_hold);
                        }
                    }
                } else if (stackCase == 7){
                    manualSetPosition(terminal);
                }
                else{
                    if (stackToggle == 5) {
                    manualSetPosition(stack_five);
                    } else if (stackToggle == 4) {
                    manualSetPosition(stack_four);
                    } else if (stackToggle == 3) {
                    manualSetPosition(stack_three);
                    } else if (stackToggle == 2) {
                    manualSetPosition(stack_two);
                    } else if (stackToggle == 1) {
                    manualSetPosition(stack_one);
                    }
                    stackCase = 0;
                    if(time.time() > 0.2){
                        GrabberOpen();
                    }
                }
                if(gamepad2.isPress(GamepadEx.Control.right_bumper)){
                    stackCase = 0;
                    mRobotState = ARM_STATE.NORMAL;
                }
                break;
        }
        if(gamepad2.isPress(GamepadEx.Control.dpad_down)){
            stackCase -= 1;
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_up)){
            stackCase += 1;
        }
    }
    public void write(){
        rightArm.write();
        leftArm.write();
        grabber.write();
    }
}

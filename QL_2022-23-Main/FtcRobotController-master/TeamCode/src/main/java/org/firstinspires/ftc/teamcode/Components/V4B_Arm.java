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
    public Caching_Servo grabber;
    Robot robot;

    private double rightArmPosition;
    private double leftArmPosition;
    ElapsedTime time = new ElapsedTime();
    ElapsedTime secondTime = new ElapsedTime();


    private double front_hold = 0.33;
    private double auto_hold = .38;
    private double hold = 0.7;

    private double out = 1;
    private double front = 0.0;
    private double hover = 0.11;
    private double terminal = 0.02;
    public static double grabberOpen = 0.285;
    public static double grabberClose = 0.46;

    private double stack_five = 0.08;
    private double stack_four = 0.15;
    private double stack_three = 0.09;
    private double stack_two = 0.04;
    private double stack_one = 0;


    public static boolean slideToggle;
    public static boolean armToggle = false;
    public static int grabberToggle = 5;
    public static int stackToggle = 5;
    public static int stackCase = 0;
    private int dropToggle = 0;
    private boolean tipped = false;

    public V4B_Arm(HardwareMap map){
        mRobotState = ARM_STATE.NORMAL;
        rightArm = new Caching_Servo(map, "rightarm");
        leftArm = new Caching_Servo(map, "leftarm");
        grabber = new Caching_Servo(map,"grabber");
        leftArm.setZeros(.01, .93);
        rightArm.setZeros(.01, .92);
        grabberToggle = 5;
        stackToggle = 5;
        stackCase = 0;
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

    public void V4BAutoHold(){
        manualSetPosition(auto_hold);
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

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry) {

        switch(mRobotState){
            case NORMAL:
                slideToggle = false;
                if(gamepad.isPress(GamepadEx.Control.right_bumper)){
                    grabberToggle += 1;
                    tipped = false;
                    time.reset();
                }

                if(gamepad.isPress(GamepadEx.Control.dpad_down)){
                    tipped = true;
                }

                if(gamepad.isPress(GamepadEx.Control.x)){
                    grabberToggle = 7;
                    time.reset();
                }

                if(gamepad.isPress(GamepadEx.Control.left_bumper)){
                    time.reset();
                    grabberToggle = 0;
                }

                telemetry.addData("tipped", tipped);

                if(grabberToggle == 1){
                    if(time.time() > 0.15) {
                        GrabberClose();
                    }

                    if(time.time() > 0.35){
                        manualSetPosition(hold);
                    }else{
                        manualSetPosition(front);
                    }
                } else if(grabberToggle == 2){
                    manualSetPosition(out);
                } else if(grabberToggle == 3){
                    if(time.time() > 0.1){
                        if(time.time() > 0.25){ //
                            GrabberClose();
                        }else{
                            GrabberOpen();
                        }
                        if(time.time() > 0.35){
                            manualSetPosition(front_hold);
                        }
                    }
                } else if (grabberToggle == 5){
                    manualSetPosition(front_hold);
                    grabber.setPosition(grabberClose);
                }
                else if (grabberToggle == 7){
                    manualSetPosition(terminal);
                } else if(grabberToggle == 8){
                    grabber.setPosition(grabberOpen);
                }
                else{
                    grabberToggle = 0;
                    if(tipped) {
                        manualSetPosition(front);
                    }else{
                        manualSetPosition(hover);
                    }
                    if(time.time() > 0.1){
                        grabber.setPosition(grabberOpen + 0.025);
                    }
                }

                if(gamepad2.isPress(GamepadEx.Control.left_bumper)){
                    grabberToggle = 0;
                    mRobotState = ARM_STATE.STACK;
                }
                break;
            case STACK:
                slideToggle = true;
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
                    manualSetPosition(stack_one);
                    if(time.time() > 0.2){
                        GrabberClose();
                    }
                    if(time.time() > 0.5){
                        manualSetPosition(hold);
                    }
                } else if(stackCase == 2){
                    manualSetPosition(out);
                } else if(stackCase == 3){
                    if(time.time() > 0.2){
                        if(time.time() > 0.39){
                            GrabberClose();
                        }else{
                            GrabberOpen();
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
                        grabberPos(grabberClose);
                        manualSetPosition(stack_five);
                    } else if (stackToggle == 4) {
                        grabberPos(grabberClose);
                        manualSetPosition(stack_five);
                    } else if (stackToggle == 3) {
                        grabberPos(grabberClose);
                        manualSetPosition(stack_five);
                    } else if (stackToggle == 2) {
                        manualSetPosition(stack_five + 0.05);
                        grabberPos(grabberClose);
                    } else if (stackToggle == 1) {
                        manualSetPosition(stack_one);
                        grabberPos(grabberClose);
                    }

                    stackCase = 0;

                    if(time.time() > 0.2){
                        if(stackToggle == 5){
                            grabberPos(grabberOpen);
                        } else if(stackToggle == 4){
                            grabberPos(grabberOpen);
                        }else if(stackToggle == 3){
                            grabberPos(grabberOpen);
                        }else if(stackToggle == 2){
                            grabberPos(grabberOpen + 0.03);
                        }else if(stackToggle == 1){
                            grabberPos(grabberOpen);
                        }
                    }
                }

                if(gamepad2.isPress(GamepadEx.Control.left_bumper)){
                    stackCase = 0;
                    mRobotState = ARM_STATE.NORMAL;
                }
            break;
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_down) && (stackCase == 2 || stackCase == 3 || stackCase == 0)){
            if(stackToggle > 1){
                gamepad2.gamepad.rumble(500);
                stackToggle -= 1;
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_up) && (stackCase == 2 || stackCase == 3 || stackCase == 0)){
            if(stackToggle < 5){
                gamepad2.gamepad.rumble(500);
                stackToggle += 1;

            }
        }

        telemetry.addData("Case", mRobotState);
        telemetry.addData("GrabberToggle", grabberToggle);
        telemetry.addData("Left servo pos", leftArm.servo.getPosition());
        telemetry.addData("StackToggle", stackToggle);
        telemetry.addData("StackCase", stackCase);
    }
    public void write(){
        rightArm.write();
        leftArm.write();
        grabber.write();
    }
}

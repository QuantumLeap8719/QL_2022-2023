package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class V4B_Arm {
    Caching_Servo left_servo;
    Caching_Servo right_servo;
    public Caching_Servo release_servo;

    private boolean out = false;
    private boolean release = false;
    ElapsedTime time = new ElapsedTime();

    public Caching_Servo front;

    public static boolean partialToggle;
    public static int goal = 2;

    /*
         100 degree angle:
            Right:0.6
            Left: 0.385;

          150 degree angle:
          Right: 0.185
          Left:0.8

          Low Level:
          Right:0.1
          Left:0.89

          Low Level 2:
          Right:0.05
          Left:0.945
     */

    public V4B_Arm(HardwareMap map){
        left_servo = new Caching_Servo(map, "left_arm");
        right_servo = new Caching_Servo(map, "right_arm");
        release_servo = new Caching_Servo(map, "release_arm");
        front = new Caching_Servo(map, "frontGate");
        partialToggle = false;
        goal = 2;

        close();
    }

    public void start(){
        time.startTime();
    }

    public void reset(){
        left_servo.setPosition(0.97);
        right_servo.setPosition(0.03);
    }

    public void V4BOutPose(){
        left_servo.setPosition(0.035);
        right_servo.setPosition(1.0);
    }

    public void V4BPartialOutPose(){
        left_servo.setPosition(0.185);
        right_servo.setPosition(0.819);
    }

    public void V4BAutoLowGoalPos(){
        V4BOutPose();
    }

    public void release(){
        release_servo.setPosition(/*0.69*/0.76);
    }

    public void halfwayRelease(){
        release_servo.setPosition(0.68);

    }

    public void close(){
        release_servo.setPosition(/*0.45*/0.53);
    }

    public void closeFront(){
        front.setPosition(0.64);
    }

    public void openFront(){
        front.setPosition(0.1);
    }

    public boolean isOut(){
        return out;
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry){
        if(gamepad.isPress(GamepadEx.Control.left_bumper) || gamepad2.isPress(GamepadEx.Control.left_bumper)){
            if(out){
                release();
            }
            time.reset();
            out = !out;
        }

        if(gamepad2.isPress(GamepadEx.Control.y)){
            if(goal != 2) {
                goal++;
            }
            if(goal == 0){
                gamepad2.gamepad.rumble(0.0, 1.0, 400);
            }else if(goal == 2){
                gamepad2.gamepad.rumble(1.0, 0.0, 400);
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.a)){
            if(goal != 0) {
                goal--;
            }
            if(goal == 0){
                gamepad2.gamepad.rumble(0.0, 1.0, 400);
            }else if(goal == 2){
                gamepad2.gamepad.rumble(1.0, 0.0, 400);
            }
        }

        if(goal == 2){
            telemetry.addLine("MODE: High Goal");
        } else if (goal == 1) {
            telemetry.addLine("MODE: Mid Goal");
        }else{
            telemetry.addLine("MODE: Low Goal");
        }

        if(gamepad.isPress(GamepadEx.Control.a)){
            partialToggle = !partialToggle;
        }

        if(!out){
            release = false;
            if(goal == 2) {
                openFront();
                if (time.time() > 0.7) {
                    close();
                }
                if (time.time() > 0.4) {
                    reset();
                } else {
                    release();
                }
            }else{
                if(time.time() > 1.5) {
                    close();
                    openFront();
                    reset();
                }
            }
        } else{
            if(time.time() > 0.1) {
                /*if(time.time() > 0.2) {
                    if (!release) {
                        halfwayRelease();
                    } else {
                        release();
                    }
                }*/
                if(release) {
                    release();
                }

                if (partialToggle || goal == 1) {
                    V4BPartialOutPose();
                } else {
                    V4BOutPose();
                }
            }

            closeFront();
        }

        telemetry.addData("Partial Toggle: ", partialToggle);
    }

    public void write(){
        left_servo.write();
        right_servo.write();
        release_servo.write();
        front.write();
    }
}

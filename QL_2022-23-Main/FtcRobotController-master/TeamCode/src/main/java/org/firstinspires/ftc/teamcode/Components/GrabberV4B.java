package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModes.Servo_Tester;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class GrabberV4B extends LinearOpMode {
    final String name = "clamp_servo";
    final String name2 = "rotate_servo";
    final String name3 = "balance_servo";

    Servo clamp_servo;
    Servo rotate_servo;
    Servo balance_servo;


    private double pos;
    private double pos2;
    private double pos3;

    GamepadEx gamepadEx;

    private boolean servoToPosToggle = false;

    @Override
    public void runOpMode(){

        gamepadEx = new GamepadEx(gamepad1);

        clamp_servo = hardwareMap.servo.get(name);
        rotate_servo = hardwareMap.servo.get(name2);
        balance_servo = hardwareMap.servo.get(name3);


        waitForStart();
        while(opModeIsActive()) {
            if(gamepadEx.isPress(GamepadEx.Control.right_trigger)){
                servoToPosToggle = !servoToPosToggle;
            }
            if(servoToPosToggle){

                rotate_servo.setPosition(pos2);
                balance_servo.setPosition(pos3);


            }
            else{

                rotate_servo.setPosition(0);
                balance_servo.setPosition(0);
            }
            if(gamepadEx.isPress(GamepadEx.Control.right_bumper)){
                servoToPosToggle = !servoToPosToggle;
            }
            if(servoToPosToggle){

                clamp_servo.setPosition(pos);

            }
            else{
                clamp_servo.setPosition(0);
            }



            telemetry.addData("Clamp Position", clamp_servo.getPosition());
            telemetry.addData("Rotate Position", rotate_servo.getPosition());
            telemetry.addData("Balance Position", rotate_servo.getPosition());


            telemetry.update();
            gamepadEx.loop();
        }
    }

}

package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Servo_Tester extends LinearOpMode {
    //Set the hardware mapping name of the servo
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

        pos = ServoTester.pos;
        pos2 = ServoTester.pos2;
        pos3 = ServoTester.pos3;

        waitForStart();
        while(opModeIsActive()) {
            if(gamepadEx.isPress(GamepadEx.Control.a)){
                servoToPosToggle = !servoToPosToggle;
            }

            if(servoToPosToggle){
                telemetry.addData("Mode", "In set position mode...");
                telemetry.addData("    ", "You can tune this position through dashboard.");

                clamp_servo.setPosition(ServoTester.pos);
                rotate_servo.setPosition(ServoTester.pos2);
                balance_servo.setPosition(ServoTester.pos3);

            }



                telemetry.addData("Mode", "In dynamic position mode..");
                telemetry.addData("    ", "Use the Dpads to change the position dynamically");
                telemetry.addData("    ", "Press A again to go back into set position mode");

                clamp_servo.setPosition(pos);
                rotate_servo.setPosition(pos2);
                balance_servo.setPosition(pos3);

            }

            telemetry.addData("Position", clamp_servo.getPosition());
            telemetry.addData("Position", rotate_servo.getPosition());
            telemetry.addData("Position",balance_servo.getPosition());


            telemetry.update();
            gamepadEx.loop();
        }
    }


@Config
class ServoTester{
    //Set the set/start position of the servo in dashboard
    public static double pos = 0.68; //clamp
    public static double pos2 = 0; //rotate
    public static double pos3 = 0; //balance
    /* positions:
    0.68->clamp
    x->rotate
    y->balance
    x and y should be inverse
     */
}


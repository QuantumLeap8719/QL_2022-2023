package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Servo_Tester extends LinearOpMode {
    //Set the hardware mapping name of the servo
    final String name = "leftarm";
    Servo leftarm;


    private double pos;

    GamepadEx gamepadEx;
    private boolean servoToPosToggle = true;


    @Override
    public void runOpMode() {
        gamepadEx = new GamepadEx(gamepad1);
        leftarm = hardwareMap.servo.get(name);
        pos = ServoTester.pos;

        waitForStart();
        while (opModeIsActive()) {
            if (servoToPosToggle) {
                telemetry.addData("Mode", "In set position mode...");
                telemetry.addData("    ", "You can tune this position through dashboard.");
                leftarm.setPosition(ServoTester.pos);
            } else {
                if (gamepad1.dpad_up) {
                    if (pos < 1) {
                        pos += 0.0005;
                    }
                } else if (gamepad1.dpad_down) {
                    if (pos > 0) {
                        pos -= 0.0005;
                    }
                }
                telemetry.addData("Mode", "In dynamic position mode..");
                telemetry.addData("    ", "Use the Dpads to change the position dynamically");
                telemetry.addData("    ", "Press A again to go back into set position mode");
                leftarm.setPosition(pos);
            }
            telemetry.addData("Position", leftarm.getPosition());

            telemetry.update();
            gamepadEx.loop();
        }
    }
}


@Config
class ServoTester{
    //Set the set/start position of the servo in dashboard
    public static double pos = 0.68; //clamp
}


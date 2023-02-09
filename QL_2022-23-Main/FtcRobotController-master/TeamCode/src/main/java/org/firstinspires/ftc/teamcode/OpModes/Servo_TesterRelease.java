package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Servo_TesterRelease extends LinearOpMode {
    //Set the hardware mapping name of the servo
    final String name = "rightarm";
    final String name1 = "leftarm";
    final String name2 ="grabber";

    Servo servo;
    Servo servo1;
    Servo servo2;

    private double pos;
    private double pos1;
    private double pos2;

    GamepadEx gamepadEx;
    private boolean servoToPosToggle = true;
    //Left 0.77
    //Right 0.21

    @Override
    public void runOpMode() {
        gamepadEx = new GamepadEx(gamepad1);
        servo = hardwareMap.servo.get(name);
        servo1 = hardwareMap.servo.get(name1);
        servo2 = hardwareMap.servo.get(name2);
        pos = ServoTesterRelease.pos;
        pos1 = ServoTesterRelease.pos1;
        pos2 = ServoTesterRelease.pos2;

        waitForStart();
        while (opModeIsActive()) {

            if (servoToPosToggle) {
                telemetry.addData("Mode", "In set position mode...");
                telemetry.addData("    ", "You can tune this position through dashboard.");
                servo.setPosition(ServoTesterRelease.pos);
                servo1.setPosition(ServoTesterRelease.pos1);
                servo2.setPosition(ServoTesterRelease.pos2);
            } else {
                if (gamepad1.dpad_up) {
                    if (pos < 1) {
                        pos += 0.0005;
                        pos1 -= 0.0005;
                    }
                } else if (gamepad1.dpad_down) {
                    if (pos > 0) {
                        pos -= 0.0005;
                        pos1 += 0.0005;
                    }
                }

                telemetry.addData("Mode", "In dynamic position mode..");
                telemetry.addData("    ", "Use the Dpads to change the position dynamically");
                telemetry.addData("    ", "Press A again to go back into set position mode");
                servo.setPosition(pos);
                servo1.setPosition(pos1);
            }

                telemetry.addData("Position", servo.getPosition());
                telemetry.addData("Position1", servo1.getPosition());
                telemetry.addData("Position2", servo2.getPosition());

                telemetry.update();
                gamepadEx.loop();
            }
        }
    }

    @Config
    class ServoTesterRelease {
        //Set the set/start position of the servo in dashboard
        public static double pos = 0.05;
        public static double pos1 = 0.977;
        public static double pos2 = 0;
    }

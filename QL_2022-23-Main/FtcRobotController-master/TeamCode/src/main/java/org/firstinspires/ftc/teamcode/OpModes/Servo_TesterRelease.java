package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Servo_TesterRelease extends LinearOpMode {
    //Set the hardware mapping name of the servo
    final String name = "release_arm";

    Servo servo;
    private double pos;
    GamepadEx gamepadEx;
    private boolean servoToPosToggle = false;
    //Left 0.77
    //Right 0.21

    @Override
    public void runOpMode() {
        gamepadEx = new GamepadEx(gamepad1);
        servo = hardwareMap.servo.get(name);
        pos = ServoTesterRelease.pos;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepadEx.isPress(GamepadEx.Control.a)) {
                servoToPosToggle = !servoToPosToggle;
            }

            if (servoToPosToggle) {
                telemetry.addData("Mode", "In set position mode...");
                telemetry.addData("    ", "You can tune this position through dashboard.");
                servo.setPosition(ServoTesterRelease.pos);
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
                servo.setPosition(pos);
            }

                telemetry.addData("Position", servo.getPosition());

                telemetry.update();
                gamepadEx.loop();
            }
        }
    }

    @Config
    class ServoTesterRelease {
        //Set the set/start position of the servo in dashboard
        public static double pos = 0.55;
    }

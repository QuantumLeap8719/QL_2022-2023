package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Slides;
import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Arm_Tester extends LinearOpMode {
    //Set the hardware mapping name of the servo
    final String name = "grabber";

    V4B_Arm arm;
    Slides slides;

    private double pos;
    private double pos1;
    private double pos2;
    private double pos3;

    GamepadEx gamepadEx;
    private boolean servoToPosToggle = true;
    //Left 0.77
    //Right 0.21

    @Override
    public void runOpMode() {
        slides = new Slides(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        arm =  new V4B_Arm(hardwareMap);
        pos = ArmTester.pos;
        pos2 = ArmTester.pos2;
        pos3 = ArmTester.pos3;
        waitForStart();
        while (opModeIsActive()) {
        //slides.setPosition(100);
        telemetry.addData("Position", slides.getPosition());
            if (servoToPosToggle) {
                telemetry.addData("Mode", "In set position mode...");
                telemetry.addData("    ", "You can tune this position through dashboard.");
                arm.manualSetPosition(ArmTester.pos);
                arm.grabberPos(ArmTester.pos2);
                arm.flicker.setPosition(ArmTester.pos3);
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
            }

           // slides.write();
            arm.write();
            telemetry.update();
            gamepadEx.loop();
        }
    }
}

@Config
class ArmTester {
    //Set the set/start position of the servo in dashboard
    public static double pos = 0.5;
    public static double pos2 = 0.51;
    public static double pos3 = 0.5;
}

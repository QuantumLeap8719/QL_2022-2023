package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

import java.util.ArrayList;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        waitForStart();

       //robot.arm.start();
        robot.updatePos();

        while (opModeIsActive()) {
           robot.operate(gamepadEx1,gamepadEx2);
           telemetry.update();
        }
    }
}
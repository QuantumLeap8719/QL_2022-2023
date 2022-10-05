package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.OpModes.AutoRed2.FREIGHT_POS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

import java.util.ArrayList;

@TeleOp(name = "RedTeleOp")
@Disabled
public class LinearTeleOpBlue extends LinearOpMode {
    Robot robot;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    double yOffset = 0.0;

    public enum mRobotState {
        AUTOMATION_IN,
        DRIVE,
        AUTOMATION_OUT
    }

    public static mRobotState robotState = mRobotState.DRIVE;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        ArrayList<CurvePoint> points = new ArrayList<>();
        robotState = mRobotState.DRIVE;
        robot.setBlue();

        waitForStart();

        robot.arm.start();
        robot.updatePos();

        while (opModeIsActive()) {
            if (robotState != mRobotState.DRIVE) {
                points = new ArrayList<>();
            }

            robot.operate(gamepadEx1, gamepadEx2);

            if (gamepadEx2.isPress(GamepadEx.Control.dpad_up)) {
                yOffset += 0.5;
            }

            if (gamepadEx2.isPress(GamepadEx.Control.dpad_down)) {
                yOffset -= 0.5;
            }

            telemetry.addData("Y Offset", yOffset);

            telemetry.update();

            switch (robotState) {
                case DRIVE:
                    if (gamepadEx1.isPress(GamepadEx.Control.left_bumper)) {
                        robotState = robot.arm.isOut() ? mRobotState.AUTOMATION_IN : mRobotState.AUTOMATION_OUT;
                    }
                    break;
                case AUTOMATION_IN:
                    if ((Math.abs(gamepadEx1.gamepad.left_stick_x) > 0.3 || Math.abs(gamepadEx1.gamepad.left_stick_y) > 0.3)) {
                        robotState = mRobotState.DRIVE;
                    }

                    points.add(new CurvePoint(FREIGHT_POS, 1d, 1d, 10));
                    points.add(new CurvePoint(new Pose2d(7, -1 + yOffset, Math.toRadians(270)), 1d, 1d, 10));
                    points.add(new CurvePoint(new Pose2d(AutoRed2.DEPOT_POS.getX(), AutoRed2.DEPOT_POS.getY() + yOffset, AutoRed2.DEPOT_POS.getHeading()), 1d, 1d, 15));


                    RobotMovement.followCurve(points, robot, telemetry);

                    break;
                case AUTOMATION_OUT:
                    if ((Math.abs(gamepadEx1.gamepad.left_stick_x) > 0.3 || Math.abs(gamepadEx1.gamepad.left_stick_y) > 0.3)) {
                        robotState = mRobotState.DRIVE;
                    }

                    points.add(new CurvePoint(new Pose2d(AutoRed2.DEPOT_POS.getX(), AutoRed2.DEPOT_POS.getY() + yOffset, AutoRed2.DEPOT_POS.getHeading()), 1d, 1d, 10));
                    points.add(new CurvePoint(new Pose2d(5, 0 + yOffset, Math.toRadians(270)), 1d, 1d, 10)); //2nd POINT FOR FREIGHT_CYCLE
                    points.add(new CurvePoint(FREIGHT_POS, 1.0, 1.0, 15));
                    RobotMovement.followCurve(points, robot, telemetry);

                    break;
            }
        }
    }
}
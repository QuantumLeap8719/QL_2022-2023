package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "Track Width Tuner")
public class Track_Width_Tuner extends LinearOpMode {
    double maxTurn = 0.3;
    double maxMove = 1;
    Robot robot = null;
    boolean running = false;
    boolean running2 = false;
    long prevTime = 0;

    GamepadEx gamepad1ex;
    TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        packet = new TelemetryPacket();

        gamepad1ex = new GamepadEx(gamepad1);
        robot.stopAndResetEncoders();

        //robot.setStartPose(new Pose2d(0,0, Math.toRadians(90)));
        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1ex.isPress(GamepadEx.Control.a)){
                running = !running;
            }

            if(gamepad1ex.isPress(GamepadEx.Control.b)){
                running2 = !running2;
            }
            robot.updatePos();

            if(running){
                //robot.drive.setPower(0, 0, 0.3);
                //robot.drive.write();
                robot.GoTo(0, 0, 14 * Math.PI, 1.0, 1.0, 0.3);
            }else if(running2){
                //robot.drive.setPower(0, 0, -0.3);
                //robot.drive.write();
                robot.GoTo(0, 0, -14 * Math.PI, 1.0, 1.0, 0.3);
            }else{
                robot.drive.driveCentric(gamepad1, 1.0, 1.0, robot.getPos().getHeading());
                robot.drive.write();
            }

            gamepad1ex.loop();

            robot.localizer.setPacket(packet);

            robot.update();

            telemetry.addData("Pos: ", robot.getPos());

            telemetry.addData("Refresh Rate", (System.currentTimeMillis() - prevTime)/1000.0);
            telemetry.addData("Right X RAW", robot.getRawRight_X_Dist());
            //telemetry.addData("Left X RAW", robot.getRawLeft_X_Dist());
            telemetry.addData("Right Y RAW", robot.getRawRight_Y_Dist());
            telemetry.addData("Left Y RAW", robot.getRawLeft_Y_Dist());
            //telemetry.addData("Difference X", Math.abs(robot.getRawRight_X_Dist() + robot.getRawLeft_X_Dist()));
            //telemetry.addData("Sum X", robot.getRawRight_X_Dist() - robot.getRawLeft_X_Dist());
            telemetry.addData("Difference Y", Math.abs(robot.getRawRight_Y_Dist() + robot.getRawLeft_Y_Dist()));
            telemetry.addData("Sum Y", robot.getRawRight_Y_Dist() - robot.getRawLeft_Y_Dist());

            telemetry.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            prevTime = System.currentTimeMillis();
        }
    }
}


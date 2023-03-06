package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;

@Autonomous
public class PID_Tuner extends OpMode {
    Robot robot;
    TelemetryPacket packet;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        packet = new TelemetryPacket();
        robot.setStartPose(new Pose2d(0, 0, 0));
        robot.localizer.reset();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        robot.arm.V4BHoldPos();
    }

    @Override
    public void loop() {
        robot.arm.V4BHoldPos();
        robot.GoTo(new Pose2d(PID_Tuner_Constants.x, PID_Tuner_Constants.y, Math.toRadians(PID_Tuner_Constants.theta)), new Pose2d(1, 1, 1));
        robot.localizer.setPacket(packet);
        robot.update();

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

@Config
class PID_Tuner_Constants {
    public static double x = 0;
    public static double y = 0;
    public static double theta = 0;
}
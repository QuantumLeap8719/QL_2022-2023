package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;

@Autonomous
public class DuckDetectorTester extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.initializeWebcam();
    }

    @Override
    public void loop() {
        telemetry.addData("Duck Position", robot.getDuckCase());
    }
}

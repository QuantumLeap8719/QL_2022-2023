package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class Pure_Pursuit_Tester extends OpMode {
   Robot robot;

   public Pose2d PRE_LOAD_CLEAR = new Pose2d(6, -27.4, Math.toRadians(0));
   public Pose2d PRE_LOAD_CLEAR2 = new Pose2d(3.3, -46.5, Math.toRadians(0));
   public Pose2d PRE_LOAD_DEPOSIT = new Pose2d(-4, -51.2, Math.toRadians(24.190));

   @Override
   public void init() {
      robot = new Robot(hardwareMap, telemetry);
      robot.localizer.reset();
   }

   @Override
   public void loop() {
      ArrayList<CurvePoint> points = new ArrayList<>();

      points.add(new CurvePoint(new Pose2d(0, 0, 0),1.0,1.0,10));
      points.add(new CurvePoint(PRE_LOAD_CLEAR,1.0,1.0,10));
      points.add(new CurvePoint(PRE_LOAD_CLEAR2,1.0,1.0,10));
      points.add(new CurvePoint(PRE_LOAD_DEPOSIT,0.5,0.5,10));

      RobotMovement.followCurve(points, robot, telemetry);
      robot.update();
   }
}

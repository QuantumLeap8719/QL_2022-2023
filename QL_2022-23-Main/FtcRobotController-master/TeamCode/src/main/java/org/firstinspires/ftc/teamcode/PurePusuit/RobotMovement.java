package org.firstinspires.ftc.teamcode.PurePusuit;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Robot;

import java.util.ArrayList;

public class RobotMovement {
    private static int index = 0;
    private static int previous_index = 0;
    private static CurvePoint followMe = new CurvePoint(0, 0, 0, 0, 0, 0);
    private static Telemetry thisTelemetry;
    private static Pose2d followPoint = new Pose2d(0,0,0);

    public static void followCurve(ArrayList<CurvePoint> allPoints, Robot robot, Telemetry telemetry){
        thisTelemetry = telemetry;
        if(index >= allPoints.size() - 2 && robot.getPos().vec().distTo(new Vector2d(allPoints.get(allPoints.size() - 1).x, allPoints.get(allPoints.size() - 1).y)) <= allPoints.get(Range.clip(index, 0, allPoints.size() - 1)).followDistance){
            followMe = allPoints.get(allPoints.size() - 1);
        }else{
            followMe = getFollowPointPath(allPoints, new Pose2d(robot.getPos().getX(), robot.getPos().getY(), robot.getPos().getHeading()), allPoints.get(Range.clip(index, 0, allPoints.size() - 1)).followDistance);
        }

        index = getCurrentLine(followMe.toVec(), allPoints);
        telemetry.addData("Current Line: ", index);
        previous_index = index;

        robot.GoTo(followMe.x, followMe.y, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).heading, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).moveSpeed, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).moveSpeed, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).turnSpeed);
    }

    public static Pose2d getLastFollowMe(){
        return followPoint;
    }

    public static void followCurveAngled(ArrayList<CurvePoint> allPoints, Robot robot, Telemetry telemetry){
        thisTelemetry = telemetry;
        double target_angle = getFollowAngle(robot.getPos(), allPoints.get(Math.min(index + 1, allPoints.size() - 1)), 0);
        if(index >= allPoints.size() - 2 && robot.getPos().vec().distTo(new Vector2d(allPoints.get(allPoints.size() - 1).x, allPoints.get(allPoints.size() - 1).y)) <= allPoints.get(Range.clip(index, 0, allPoints.size() - 1)).followDistance){
            followMe = allPoints.get(allPoints.size() - 1);
            target_angle = Math.atan2(allPoints.get(allPoints.size() - 1).x - allPoints.get(allPoints.size() - 2).x, allPoints.get(allPoints.size() - 1).y - allPoints.get(allPoints.size() - 2).y);
        }else{
            followMe = getFollowPointPath(allPoints, new Pose2d(robot.getPos().getX(), robot.getPos().getY(), robot.getPos().getHeading()), allPoints.get(index).followDistance);
        }

        index = getCurrentLine(followMe.toVec(), allPoints);
        telemetry.addData("Current Line: ", index);
        previous_index = index;

        telemetry.addData("PURE PURESUIT POS", robot.getPos());

        telemetry.addData("PP ANGLE", Math.toDegrees(target_angle));
        followPoint = new Pose2d(followMe.x, followMe.y, target_angle);
        robot.GoTo(followMe.x, followMe.y, target_angle, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).moveSpeed, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).moveSpeed, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).turnSpeed);
    }

    public static double getFollowAngle(Pose2d p, CurvePoint followMe, double preferredAngle){
        //double relativeAngleToPoint = Math_Functions.AngleWrap(absoluteAngleToTarget - p.getHeading());

        //return relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        return Math.atan2(followMe.x-p.getX(), followMe.y-p.getY());
    }

    public static void resetIndex(){
        previous_index = 0;
        index = 0;
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Pose2d robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        double runningDistance = 0;
        double previous_dist = 0;
        Vector2d maxIntersection = new Vector2d(0, 0);

        for (int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint start = pathPoints.get(i);
            CurvePoint end = pathPoints.get(i + 1);
            runningDistance += previous_dist;

            ArrayList<Vector2d> intersections = Math_Functions.lineCircleIntersection(new Vector2d(robotLocation.getX(), robotLocation.getY()), followRadius, start.toVec(), end.toVec());

            int closestDistance = 0;

            for(int j = 0; j < intersections.size(); j++){
                thisTelemetry.addData("Line Intersection" + j, intersections.get(j));
                double dist = intersections.get(j).distTo(end.toVec());
                double closestDist = intersections.get(closestDistance).distTo(end.toVec());

                if (dist < closestDist){
                    closestDistance = j;
                }
            }

            if(intersections.size() != 0){
                followMe.setPoint(intersections.get(closestDistance));
            }

            previous_dist = Math.hypot(end.x - start.x, end.y - start.y);
        }
        return followMe;
    }


    public static int getCurrentLine(Vector2d intersection, ArrayList<CurvePoint> allPoints){
        int currentline = 0;

        for (int j = 0; j < allPoints.size() - 1; j++){
            Vector2d start = new Vector2d(allPoints.get(j).x, allPoints.get(j).y);
            Vector2d end = new Vector2d(allPoints.get(j + 1).x, allPoints.get(j + 1).y);

            if (Math.abs(start.distTo(end) - (start.distTo(intersection) + end.distTo(intersection))) <= 0.003){
                if (Math.abs(j - previous_index) < 2) {
                    currentline = j;
                    return currentline;
                }
            }
        }
        return currentline;
    }
}

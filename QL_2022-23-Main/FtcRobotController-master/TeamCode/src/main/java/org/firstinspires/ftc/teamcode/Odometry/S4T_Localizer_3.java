package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;

@Config
public class S4T_Localizer_3 {
    public static double TRACK_WIDTH = 2173.9451141751259598679139890342;//3630.9785555810659563797329424431;3648.0700280808789388264691821431;//3640.7807316872701324482543057806;//3638.20242160918/']1428008798388814;//3641.9823515076139422333093781941;//<Manual//3638.8788301173219831858161448083;//3641.5844641498842038938871687857;//3637.4862243652678989978384118788;//3625.3506599545108796454610249217;//3657.221437308662920633179998537;//3654.5953807476466475929934164414;//3647.2742533654194621476247633262;//3666.2534803291279809380641521084;//3635.8150974628029979722651323634;//3637.8841117229976373372606212872; //3622.8970212485108265523574002363;//3625.4965519856784503699158350381//3624.8068805656135705815840053968;//3605.4165033322509888404083335593;//3602.3925594135049774607995420552;//3599.2094605516670707454218667877;//3625.7087585764676441509410133892;//3617.4327015356890866909590576939;//3605.5756582753428841761772173226;//2814.7745347874879345745360180654;//2798.0632657628389243188032229113;//2805.503759352385031265998538849;//2703.5252295662530948720862674681;
    public static double AUX_WIDTH = -4.51356273;

    private final double EPSILON = 1e-6;
    private Pose2d myPose = new Pose2d(0, 0,0);

    double prevHeading = 0;

    double prevx = 0;
    double prevy = 0;

    double prevely = 0;
    double prevery = 0;
    double preverx = 0;

    double prevelyRaw = 0;
    double preveryRaw = 0;
    double preverxRaw = 0;

    private double heading = 0;
    Telemetry telemetry;
    public double TICKS_TO_INCHES_VERT = 197.958333;//303.547368;
    public double TICKS_TO_INCHES_STRAFE = 197.958333;//335.381388888888888;

    public final Vector2d DASHBOARD_OFFSET_FROM_CENTER = new Vector2d(-48, -55);

    private TelemetryPacket packet;
    FtcDashboard dashboard;
    public boolean start;

    public S4T_Localizer_3(Telemetry telemetry){
        this.telemetry = telemetry;

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        start = false;
    }

    public void setPacket(TelemetryPacket packet){
        this.packet = packet;
    }

    private void addPacket(String caption, Object value){
        if(packet != null){
            packet.put(caption, value);
        }
    }

    double dtheta = 0;
    public Pose2d dashboardPos = new Pose2d(0, 0, 0);

    public void update(double elyRaw, double erxRaw, double eryRaw){
        heading = (eryRaw - elyRaw) / TRACK_WIDTH;

        dtheta = heading - prevHeading;


        double y = ((elyRaw + eryRaw)/2) / TICKS_TO_INCHES_VERT;
        double x = erxRaw / TICKS_TO_INCHES_STRAFE;
        double dy = y - prevy;
        double dx = (x - prevx) - (dtheta * AUX_WIDTH);

        prevx = x;
        prevy = y;

        prevely = elyRaw;
        preverx = erxRaw;
        prevery = eryRaw;

        prevelyRaw = elyRaw;
        preverxRaw = erxRaw;
        preveryRaw = eryRaw;

        Vector2 myVec = ConstantVelo(dy, dx, prevHeading, dtheta);

        prevHeading = heading;

        heading %= 2 * Math.PI;

        if(myVec.magnitude() < 5) {
            myPose = myPose.plus(new Pose2d(myVec.x, myVec.y, dtheta));
        }
        myPose = new Pose2d(myPose.getX(), myPose.getY(), (Math.toRadians(360) - heading) % Math.toRadians(360));

        addPacket("X Pos: ", myPose.getX());
        addPacket("Y Pos: ", myPose.getY());
        addPacket("Theta Pos: ", Math.toDegrees(myPose.getHeading()));
        addPacket("D Theta: ", Math.toDegrees(dtheta));
        addPacket("Raw Right Y: ", eryRaw);
        addPacket("Raw Left Y: ", elyRaw);
        addPacket("Raw Right X: ", erxRaw);


        dashboard.sendTelemetryPacket(packet);

        dashboardPos = new Pose2d(myPose.getY() + DASHBOARD_OFFSET_FROM_CENTER.getY(), -myPose.getX() + DASHBOARD_OFFSET_FROM_CENTER.getX(), (2 * Math.PI) - myPose.getHeading());
    }

    public void reset(){
        myPose = new Pose2d(0, 0, 0);
        heading = 0;
    }


    public double angleWrap(double angle){
        return ((2 * Math.PI) + angle) % (2 * Math.PI);
    }

    public void setHeading(double heading){
        this.heading = heading;
    }

    public static double normalizationFactor = 8;

    public Pose2d getPose(){
        return new Pose2d(myPose.getX(), myPose.getY(), myPose.getHeading());
    }

    public Vector2 ConstantVelo(double delta_y, double delta_x, double prev_heading, double delta_theta){
        Pose2d RawRobotDelta = new Pose2d(delta_x, delta_y, delta_theta);

        double sinterm = 0;
        double costerm = 0;

        if(delta_theta <= EPSILON){
            sinterm = 1.0 - delta_theta * delta_theta / 6.0;
            costerm = delta_theta / 2.0;
        }else{
            sinterm = Math.sin(delta_theta) / delta_theta;
            costerm = (1 - Math.cos(delta_theta)) / delta_theta;
        }

        Vector2 FeildCentricDelta = new Vector2((sinterm * -RawRobotDelta.getX()) - (costerm * RawRobotDelta.getY()), (costerm * RawRobotDelta.getX()) + (sinterm * RawRobotDelta.getY()));
        FeildCentricDelta.rotate(prev_heading);

        return FeildCentricDelta;
    }
}

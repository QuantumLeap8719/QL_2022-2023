package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Components.Robot;

@Config
public class S4T_Localizer {

    public static double TRACK_WIDTH1 = 3644.276835270522099990644119116;//3630.9785555810659563797329424431;3648.0700280808789388264691821431;//3640.7807316872701324482543057806;//3638.20242160918/']1428008798388814;//3641.9823515076139422333093781941;//<Manual//3638.8788301173219831858161448083;//3641.5844641498842038938871687857;//3637.4862243652678989978384118788;//3625.3506599545108796454610249217;//3657.221437308662920633179998537;//3654.5953807476466475929934164414;//3647.2742533654194621476247633262;//3666.2534803291279809380641521084;//3635.8150974628029979722651323634;//3637.8841117229976373372606212872; //3622.8970212485108265523574002363;//3625.4965519856784503699158350381//3624.8068805656135705815840053968;//3605.4165033322509888404083335593;//3602.3925594135049774607995420552;//3599.2094605516670707454218667877;//3625.7087585764676441509410133892;//3617.4327015356890866909590576939;//3605.5756582753428841761772173226;//2814.7745347874879345745360180654;//2798.0632657628389243188032229113;//2805.503759352385031265998538849;//2703.5252295662530948720862674681;
    public static double TRACK_WIDTH2 = 1961.3141102043537912806852811685;//1950.4279120968681503140936317538;//1966.6139698093139059617891104888;//1959.7941804978261908240924412283;//1953.9929828221266058353166280534;//1957.4625605815299241550782940949;//1957.0328422351818067485023079338;//1961.2663637214262226799546160395;//<Manual//1956.8100253148531532784258706651;//1954.9399547335233830831414864454;//1966.9163642011885070997499896392;//1947.6188273512961976377728333303;//1811.0240974426770257141283434158;//1811.8198721581365023929727622327;//1812.4564919305040837360482972862;//1817.7881825240825774843059033592;//1826.9594861197530462079878302235;//1826.1438170364070826121723009362;//1821.6079011582880655427591136801;//1822.642408288385385225256858142;//1825.7724555025259934953782388217;//1810.5466326134013397068216921257;//1809.1672897732715801301580328432;//1806.9656463938336946520218074498;//1822.9209294387962020628524047279;//1819.857196784277216849301392283;//1811.4219848004067640535505528243;//2577.4347259016990151091881059362;//2570.2329647267907511656461156436;//2572.3019789869853905306416045674;//2587.620642259580316598396666792;//2584.6364870766072790527300962288;//2853.4889746945914750003169935058;

    private final double EPSILON = 1e-6;
    private static Pose2d myPose = new Pose2d(0, 0,0);
    double prevHeading = 0;

    double prevx = 0;
    double prevy = 0;

    double prevely = 0;
    double prevery = 0;
    double prevelx = 0;
    double preverx = 0;

    double prevelyRaw = 0;
    double preveryRaw = 0;
    double prevelxRaw = 0;
    double preverxRaw = 0;

    private static double heading = 0;
    Telemetry telemetry;
    public static double k_strafe = 1.0;
    public static double k_vert = 0.6; //Gear Ratio Offset!!! //0.6
    public double TICKS_TO_INCHES_VERT = 333.791666;//303.547368;
    public double TICKS_TO_INCHES_STRAFE = 197.958333;//335.381388888888888;

    public static double clipping_strafe = 0.0;
    public static double clipping_vert = 0.0;
    public final Vector2d DASHBOARD_OFFSET_FROM_CENTER = new Vector2d(-48, -55);

    public boolean blue;

    private TelemetryPacket packet;
    FtcDashboard dashboard;

    public S4T_Localizer(Telemetry telemetry){
        blue = false;
        this.telemetry = telemetry;

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    public void setPacket(TelemetryPacket packet){
        this.packet = packet;
    }

    private void addPacket(String caption, Object value){
        if(packet != null){
            packet.put(caption, value);
        }
    }

    public double wf = 1;
    public double ws = 1;
    double dtheta = 0;
    public Pose2d dashboardPos = new Pose2d(0, 0, 0);

    public void update(double elxRaw, double elyRaw, double erxRaw, double eryRaw){
        double y = ((elyRaw + eryRaw)/2) / TICKS_TO_INCHES_VERT;
        double x = ((elxRaw + erxRaw)/2) / TICKS_TO_INCHES_STRAFE;
        double dy = y - prevy;
        double dx = x - prevx;

        addPacket("dx", dx);
        addPacket("dy", dy);

        prevx = x;
        prevy = y;

        double dElyRaw = elyRaw - prevelyRaw;
        double dEryRaw = eryRaw - preveryRaw;
        double dElxRaw = elxRaw - prevelxRaw;
        double dErxRaw = erxRaw - preverxRaw;

        prevelx = elxRaw;
        prevely = elyRaw;
        preverx = erxRaw;
        prevery = eryRaw;

        prevelxRaw = elxRaw;
        prevelyRaw = elyRaw;
        preverxRaw = erxRaw;
        preveryRaw = eryRaw;

        double dthetastrafe = -((dErxRaw - dElxRaw) / TRACK_WIDTH2);
        double dthetavert = (dEryRaw - dElyRaw) / TRACK_WIDTH1;

        dtheta = weightedTheta(dx, dy, dthetavert, dthetastrafe);
        heading += dthetavert;
        heading %= 2 * Math.PI;

        telemetry.addData("Vertical Heading", Math.toDegrees(((eryRaw - elyRaw) / TRACK_WIDTH1) % (2 * Math.PI)));
        telemetry.addData("Strafe Heading", Math.toDegrees(((erxRaw - elxRaw) / TRACK_WIDTH2) % (2 * Math.PI)));

        Vector2 myVec = ConstantVelo(dy, dx, prevHeading, dtheta);
        prevHeading = heading;

        myPose = myPose.plus(new Pose2d(myVec.x, myVec.y, dtheta));
        myPose = new Pose2d(myPose.getX(), myPose.getY(), (Math.toRadians(360) - heading) % Math.toRadians(360));

        addPacket("X Pos: ", myPose.getX());
        addPacket("Y Pos: ", myPose.getY());
        addPacket("Theta Pos: ", Math.toRadians(myPose.getHeading()));

        dashboard.sendTelemetryPacket(packet);

        dashboardPos = new Pose2d(myPose.getY() + DASHBOARD_OFFSET_FROM_CENTER.getY(), -myPose.getX() + DASHBOARD_OFFSET_FROM_CENTER.getX(), (2 * Math.PI) - myPose.getHeading());
    }

    public void blue(){
        blue = true;
    }

    public void reset(){
        myPose = new Pose2d(0, 0, 0);
        heading = Robot.startPos.getHeading();
    }

    public double angleWrap(double angle){
        return ((2 * Math.PI) + angle) % (2 * Math.PI);
    }

    public double weightedTheta(double dx, double dy, double dthetavert, double dthetastrafe){
        determineWeights(dx, dy);

        if(Math.abs(wf) <= clipping_vert){
            wf = 0;
        }

        if(Math.abs(ws) <= clipping_strafe){
            ws = 0;
        }

        double value = 0;
        double total = wf + ws;
        if(total != 0){
            value = ((wf * dthetavert) + (ws * dthetastrafe))/total;
        }else{
            value = (dthetavert + dthetastrafe)/2;
            //value = dthetavert;
        }

        //value = Math.abs(ws) > Math.abs(wf) ? dthetastrafe : dthetavert;
        //value = Math.abs(ws) == Math.abs(wf) ? (dthetastrafe + dthetavert)/2 : value;

        addPacket("wf", wf);
        addPacket("ws", ws);

        return value;
    }

    private double prevdx = 0;
    private double prevdy = 0;

    public void setHeading(double heading){
        this.heading = heading;
    }

    public static double normalizationFactor = 8;

    public void determineWeights(double dx, double dy){
        double total = dx + dy;

        double mydx = (dx + prevdx)/2;
        double mydy = (dy + prevdy)/2;

        //double mydx = Math.abs(dx) < 0.01 ? 0.0 : dx;
        //double mydy = Math.abs(dy) < 0.01 ? 0.0 : dy;
        //double mydx = dx;
        //double mydy = dy;

        if(total != 0){
            mydx /= total;
            mydy /= total;
            mydx *= normalizationFactor;
            mydy *= normalizationFactor;
        }

        //If dx is higher, wf is lower and vice versa
        if(mydx != 0) {
            wf = Math.pow(Math.E, -k_strafe * Math.abs(mydx));
        }else{
            ws = 0;
        }

        //If dy is high, ws is lower and vice versa
        if(mydy != 0) {
            ws = Math.pow(Math.E, -k_vert * Math.abs(mydy));
        }else{
            wf = 0;
        }

        prevdx = dx;
        prevdy = dy;
    }

    public Pose2d getPose(){
        return new Pose2d(myPose.getX(), myPose.getY(), myPose.getHeading());
    }

    private Vector2 circleUpdate(double dr, double dx, double dy, double dtheta){
        if(dtheta <= EPSILON){
            double sineTerm = 1.0 - dtheta * dtheta / 6.0;
            double cosTerm = dtheta / 2.0;


            return new Vector2(
                    sineTerm * dx - cosTerm * dy,
                    cosTerm * dx + sineTerm * dy
            );
        }else{
            double radius = (TRACK_WIDTH1/2) + (dr/dtheta);
            double strafe_radius = dy/dtheta;
            return new Vector2(
                    (radius * (1 - Math.cos(dtheta))) + (strafe_radius * Math.sin(dtheta)),
                    (Math.sin(dtheta) * radius) + (strafe_radius * (1 - Math.cos(dtheta)))
            );
        }
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

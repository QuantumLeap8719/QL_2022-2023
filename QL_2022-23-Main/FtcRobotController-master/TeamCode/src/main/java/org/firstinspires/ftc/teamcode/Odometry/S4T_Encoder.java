package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class S4T_Encoder {
    public DcMotor encoder;
    public double distance;
    public double offset;
    boolean reverse = false;

    public S4T_Encoder(HardwareMap hardwareMap, String name){
        encoder = hardwareMap.get(DcMotorEx.class, name);
        //encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reset(){
        offset = encoder.getCurrentPosition();
        distance = 0;
    }

    public void update(){
        double val = encoder.getCurrentPosition();
        if(val != 0) {
            distance = val - offset;
        }
    }
}

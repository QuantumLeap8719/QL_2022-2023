package org.firstinspires.ftc.teamcode.Wrapper;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Caching_Motor {
    HardwareMap hardwareMap;
    String name;
    public DcMotorEx motor;
    double prev_power = 0.0;

    double query = -2.0;

    double EPSILON = 0.0075;

    public Caching_Motor(HardwareMap hardwareMap, String name){
        this.hardwareMap = hardwareMap;
        this.name = name;

        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    public void setPower(double power){
        if (Math.abs(prev_power - power) > EPSILON){
            query = power;
        }
        else{
            query = -2.0;
        }
    }

    public void write(){
        if (query != -2.0) {
            motor.setPower(query);
            prev_power = query;
        }
    }
}
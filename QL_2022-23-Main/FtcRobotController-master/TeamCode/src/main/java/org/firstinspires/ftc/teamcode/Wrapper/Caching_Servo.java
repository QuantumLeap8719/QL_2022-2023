package org.firstinspires.ftc.teamcode.Wrapper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Caching_Servo {
    public Servo servo;
    double prev_pos = -1.0;

    public double query = -2.0;

    public double m = 1;
    public double b = 0;

    double EPSILON = 0.001;

    public Caching_Servo(HardwareMap map, String name){
        servo = map.servo.get(name);
    }

    public void setPosition(double pos){
        double newPos = m * pos + b;
        if (Math.abs(newPos - prev_pos) > EPSILON){
            query = newPos;
        }
        else{
            query = -1.0;
        }
    }

    public void setZeros(double min, double max){
        m = (max - min);
        b = min;
    }

    public void setPosition(double pos, double EPSILON){
        double newPos = m * pos + b;
        if (Math.abs(newPos - prev_pos) > EPSILON){
            query = newPos;
        }
        else{
            query = -1.0;
        }
    }

    public double getPosition(){
        return prev_pos;
    }

    public void write(){
        if (query != -1.0) {
            servo.setPosition(query);
            prev_pos = query;
        }
    }
}
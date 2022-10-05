package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel {
    CRServo left;
    Telemetry telemetry;

    public Carousel(HardwareMap map, Telemetry telemetry){
        left = map.get(CRServo.class, "carousel_right");
    }

    public void write(){

    }

    public void runCarousel(){
        left.setPower(0.5);
    }

    public void operate(Gamepad gamepad){
        if(gamepad.dpad_left){
            left.setPower(1);
        }else if(gamepad.dpad_right){
            left.setPower(-1.0);
        }else{
            left.setPower(0);
        }
    }

    public void stopCarousel(){
        left.setPower(0);
    }
}

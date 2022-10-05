package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Slides;

@TeleOp
public class Slide_Tester extends OpMode {
    Slides slides;

    public void init(){
        slides = new Slides(hardwareMap, telemetry);
    }

    public void loop(){
        telemetry.addData("Position", slides.getPosition());

        slides.write();
        telemetry.update();
    }
}

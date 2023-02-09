package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Slides;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Slide_Tester extends OpMode {
    Slides slides;
    boolean val = false;

    public void init(){
        slides = new Slides(hardwareMap, telemetry);
    }

    public void loop(){
            slides.setPosition(250);

        telemetry.addData("Position", slides.getPosition());

        slides.write();
        telemetry.update();
    }
}

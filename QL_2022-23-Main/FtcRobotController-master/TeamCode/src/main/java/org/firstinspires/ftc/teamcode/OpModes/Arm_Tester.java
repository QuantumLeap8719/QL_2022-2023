package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Arm_Tester extends OpMode {
    V4B_Arm arm;
    GamepadEx gamepadEx;
    boolean out;
    boolean release;

    public void init(){
        arm = new V4B_Arm(hardwareMap);
        gamepadEx = new GamepadEx(gamepad1);
    }

    public void loop(){
        if(gamepadEx.isPress(GamepadEx.Control.a)){
            out = !out;
        }

        if(!out){
            arm.reset();
        } else{
            arm.V4BOutPose();
        }

        if(gamepadEx.isPress(GamepadEx.Control.b)){
            release = !release;
        }

        if(!release){
            arm.close();
        } else{
            arm.release();
        }

        arm.write();
        gamepadEx.loop();
    }
}

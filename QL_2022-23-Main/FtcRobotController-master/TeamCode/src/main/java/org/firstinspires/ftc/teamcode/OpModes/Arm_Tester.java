package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Arm_Tester extends OpMode {
    V4B_Arm arm;
    GamepadEx gamepadEx;
    double pos = 0;
    public void init(){
        arm = new V4B_Arm(hardwareMap);
        gamepadEx = new GamepadEx(gamepad1);
    }

    public void loop(){
        if(gamepadEx.isPress(GamepadEx.Control.dpad_up) && pos < 1.0){
            pos += 0.2;
        }

        if(gamepadEx.isPress(GamepadEx.Control.dpad_down) && pos > 0.0){
            pos -= 0.2;
        }

        arm.manualSetPosition(pos);

        arm.write();
        gamepadEx.loop();
    }
}

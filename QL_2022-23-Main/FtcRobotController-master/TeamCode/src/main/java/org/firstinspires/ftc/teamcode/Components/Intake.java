package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    Caching_Motor intake;
    Caching_Servo intake_dropper;
    double power = 1.0;
    boolean intakeToggle = false;
    private DistanceSensor sensorRange;
    private boolean outtake = false;
    ElapsedTime time;
    Telemetry telemetry;

    public Intake(HardwareMap map, Telemetry telemetry){
        intake = new Caching_Motor(map, "intakeL");
        intake_dropper = new Caching_Servo(map, "intake_dropper");
        sensorRange = map.get(DistanceSensor.class, "sensor_range");
        this.telemetry = telemetry;

        intake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        time = new ElapsedTime();
        time.startTime();
    }

    public boolean isBlockIn(){
        return !(sensorRange.getDistance(DistanceUnit.INCH) > 2.2);
    }

    public void intake(GamepadEx gamepadEx, GamepadEx gamepad2Ex, Telemetry telemetry){
        /*intakeR.setPower(gamepadEx.gamepad.right_trigger - gamepadEx.gamepad.left_trigger);
        intakeL.setPower(-gamepadEx.gamepad.right_trigger + gamepadEx.gamepad.left_trigger);*/

        //gamepadEx.gamepad.rumble(400);

        clamp();
        if(gamepadEx.isPress(GamepadEx.Control.right_trigger)){
            intakeToggle = !intakeToggle;
        }

        if(gamepadEx.isPress(GamepadEx.Control.left_bumper)){
            outtake = true;
            intakeToggle = false;
        }

        if(Math.abs(gamepad2Ex.gamepad.left_trigger + gamepad2Ex.gamepad.right_trigger) > 0.1) {
            intake.setPower(gamepad2Ex.gamepad.left_trigger + gamepad2Ex.gamepad.right_trigger);
        }else{
            if(Math.abs(gamepadEx.gamepad.left_trigger) < 0.1) {
                if (intakeToggle) {
                    intake.setPower(-1.0);
                    if(isBlockIn()){
                        gamepadEx.gamepad.rumble(400);
                    }
                }else{
                    if(outtake){
                        if(time.time() > 1.0){
                            outtake = false;
                        }

                        intake(false);
                    }else{
                        intake.setPower(0.0);
                        time.reset();
                    }
                }
            }else{
                intake.setPower(0.5);
            }
        }

    }

    public void intake(boolean ifTrue){
        if(ifTrue){
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.5);
        }
    }

    public void stop(){
        intake.setPower(0.0);
    }

    public void clamp(){
        intake_dropper.setPosition(0.1);
    }

    public void drop(){
        intake_dropper.setPosition(0.04);
    }

    public void write(){
        intake.write();
        intake_dropper.write();
    }
}

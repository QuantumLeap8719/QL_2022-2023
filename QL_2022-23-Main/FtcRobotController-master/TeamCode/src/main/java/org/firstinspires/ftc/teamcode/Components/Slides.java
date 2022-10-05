package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@Config
public class Slides {
    Caching_Motor rSlides;
    Caching_Motor lSlides;

    public static double kp = 0.01;//0.02;
    public static double ki = 0.0;
    public static double kd = 0.0;//0.0008;
    public static double gff = 0.0;//0.25;

    public static double high_goal_position = 1612;//326;

    public static double downPower = 1.0;//0.245;

    public PIDFController controller;
    Telemetry telemetry;
    public STATE mRobotState;
    DigitalChannel digitalTouch;
    ElapsedTime time;

    public enum STATE{
        AUTOMATION,
        MANUAL,
        IDLE,
        DOWN
    }

    public Slides(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        rSlides = new Caching_Motor(map, "rSlide");
        lSlides = new Caching_Motor(map, "lSlide");
        controller = new PIDFController(new PIDCoefficients(kp, ki, kd));
        reset();
        mRobotState = STATE.IDLE;
        time = new ElapsedTime();
        time.startTime();
        digitalTouch = map.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        setBrake();
    }

    public void write(){
        lSlides.write();
        rSlides.write();
    }

    public void setPosition(double target){
        controller.setTargetPosition(target);
        setPower(controller.update(getPosition()));

        telemetry.addData("Target", target);
        telemetry.addData("Error", controller.getLastError());
    }

    public void reset(){
        rSlides.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlides.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlides.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlides.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPosition(){
        double lSlidePos = lSlides.motor.getCurrentPosition();
        double rSlidePos = rSlides.motor.getCurrentPosition();
        telemetry.addData("Left Slide Position", lSlidePos);
        telemetry.addData("Right Slide Position", rSlidePos);
        return Math.abs((rSlidePos - lSlidePos)/2.0);
    }

    public void setCoast(){
        lSlides.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rSlides.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrake(){
        lSlides.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlides.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power){
        /*if(getPosition() > 50 || mRobotState == STATE.MANUAL) {
            power += gff;
            telemetry.addLine("Gff on");
        }else{
            telemetry.addLine("Gff Off");
        }
        if(power < 0 && mRobotState != STATE.IDLE) { //Change the 30% for PID it will mess with the constants.
            rSlides.setPower(power * 0.3);
            lSlides.setPower(-power * 0.3);
        }else {
            rSlides.setPower(power);
            lSlides.setPower(-power);
        }*/
        if(getPosition() > 50 || mRobotState == STATE.MANUAL) {
            power += gff;
        }
        rSlides.setPower(power);
        lSlides.setPower(-power);
    }

    public boolean isDown(){
        return !digitalTouch.getState();
    }

    public void drop(){
        if(isDown()){
            setPower(0);
        }else{
            setPower(-0.4);
        }
    }

    boolean prevDown = false;

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2){
        if(mRobotState == STATE.MANUAL) {
            setBrake();
            double power = gamepad2.gamepad.right_stick_y;

            setPower(power);

            if(gamepad2.isPress(GamepadEx.Control.b)){
                mRobotState = STATE.DOWN;
            }

            if(gamepad2.isPress(GamepadEx.Control.left_bumper) || gamepad1.isPress(GamepadEx.Control.left_bumper)){
                time.reset();
                mRobotState = STATE.AUTOMATION;
            }
        }else if(mRobotState == STATE.AUTOMATION){
            setBrake();
            if(gamepad1.isPress(GamepadEx.Control.left_bumper)){
                time.reset();
                mRobotState = STATE.DOWN;
            }else{
                if(time.time() > 0.75 && V4B_Arm.goal == 2) {
                    setPosition(V4B_Arm.partialToggle ? 818.71 : high_goal_position);
                }else if(V4B_Arm.goal == 1){
                    setPosition(98.5);
                }
            }

            if(gamepad2.gamepad.right_stick_y >= 0.1){
                mRobotState = STATE.MANUAL;
            }
        }else if(mRobotState == STATE.IDLE){
            setBrake();
            if(!isDown()){
                telemetry.addLine("not down");
                setPower(-0.5);
            }else{
                reset();
                telemetry.addLine("down");
                setPower(0.0);
            }

            if(gamepad2.isPress(GamepadEx.Control.left_bumper) || gamepad1.isPress(GamepadEx.Control.left_bumper)){
                time.reset();
                mRobotState = STATE.AUTOMATION;
            }

            if(gamepad2.gamepad.right_stick_y >= 0.1){
                time.reset();
                mRobotState = STATE.MANUAL;
            }
        }else if(mRobotState == STATE.DOWN){
            setCoast();
            if(time.time() > 0.75) {
                if (getPosition() < 300 || isDown()) {
                    mRobotState = STATE.IDLE;
                } else {
                    setPower(-downPower);
                }
            }
        }

        telemetry.addData("Slide Position", getPosition());
        telemetry.addData("Slide State", mRobotState);
    }
}

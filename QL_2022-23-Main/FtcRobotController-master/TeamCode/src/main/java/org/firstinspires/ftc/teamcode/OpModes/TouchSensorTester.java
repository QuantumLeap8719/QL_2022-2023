package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TouchSensorTester extends OpMode {
    TouchSensor touch;

    public void init() {
        touch = hardwareMap.touchSensor.get("sensor_digital");
    }

    public void loop(){
        telemetry.addData("Sensor output: ", touch.isPressed());
        telemetry.addData("Value: ", touch.getValue());
    }
}

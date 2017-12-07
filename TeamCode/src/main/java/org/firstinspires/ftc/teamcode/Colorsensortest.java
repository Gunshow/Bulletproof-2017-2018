package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorHTColor;

/**
 * Created by brybr on 12/1/2017.
 */
@TeleOp(name="color sensor maybe ", group="Linear Opmode")
@Disabled

public class Colorsensortest extends LinearOpMode {

    private ColorSensor color_sensor;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        color_sensor = hardwareMap.colorSensor.get("color");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            color_sensor.enableLed(true);
            color_sensor.red();   // Red channel value
            color_sensor.green(); // Green channel value
            color_sensor.blue();  // Blue channel value

            color_sensor.alpha(); // Total luminosity
            color_sensor.argb();  // Combined color value
        }
    }
}
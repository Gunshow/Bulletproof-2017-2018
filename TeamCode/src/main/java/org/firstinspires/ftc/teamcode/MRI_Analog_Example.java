/*
Modern Robotics Analog Example
Created 7/25/2017 by Colton Mehlhoff of Modern Robotics using FTC SDK 3.10
Reuse permitted with credit where credit is due

The Modern Robotics Core Device Interface reads analog input
using a 10 bit value meaning the Android Phone reads a value from 0 to 1023.
The FTC SDK supplies a "voltage". The SDK is scaling this value of 0-1023 to 1-5 with decimal places.

Configuration:
Analog Input on Core Device Interface "light"
Core Device Interface named "Device Interface Module 1"

Support is available by emailing support@modernroboticsinc.com
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;


@TeleOp(name = "Analog Example", group = "MRI")
//@Disabled
public class MRI_Analog_Example extends LinearOpMode {

    //An Analog Input. In this example, we used a Light Sensor although it could be any analog sensor.
    AnalogInput MRLightSensor;

    //CDI. Using this, we can read any analog sensor on this CDI without creating an instance for each sensor.
    DeviceInterfaceModule cdi;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Link objects to configuration file
        MRLightSensor = hardwareMap.analogInput.get("light");
        cdi = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        waitForStart();

        while (opModeIsActive()) {

            //Read the light sensor using the Analog Input object
            telemetry.addData("light", MRLightSensor.getVoltage());

            //Read each Analog Port of the CDI. 0-7
            for (int i = 0; i < 8; i++) {
                telemetry.addData("Analog " + i, cdi.getAnalogInputVoltage(i));
            }
            telemetry.update();
        }
    }
}

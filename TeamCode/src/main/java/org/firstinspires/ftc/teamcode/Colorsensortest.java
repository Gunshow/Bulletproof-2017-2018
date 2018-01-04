/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.AutonomousBlueside.DRIVE_SPEED;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *d
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Colorsensortest", group="Linear Opmode")
//@Disabled
public class Colorsensortest extends LinearOpMode {
    float hsvValues[] = {0F, 0F, 0F};

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor LeftDriveFront = null;
    public DcMotor RightDriveFront = null;
    public DcMotor LeftDriveBack = null;
    public DcMotor RightDriveBack = null;
    public DcMotor Pulley = null;
    public Servo   TopServo = null;
    public Servo   BottomServo = null;
    public Servo   ColorSensorArm =null;
    public int     servovaluetop = 1;
    public int     servovaluebottom = 1;
    public int     LiftCountsPerInch = 475;
    public ColorSensor ColorSensor = null;  // Hardware Device Object

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        LeftDriveFront = hardwareMap.get(DcMotor.class, "left_drive");
        RightDriveFront = hardwareMap.get(DcMotor.class, "right_drive");
        LeftDriveBack = hardwareMap.get(DcMotor.class, "left_drive2");
        RightDriveBack =  hardwareMap.get (DcMotor.class, "right_drive2");
        Pulley =  hardwareMap.get (DcMotor.class, "pulley");
        TopServo  =  hardwareMap.get (Servo.class, "top_servo");
        BottomServo =  hardwareMap.get (Servo.class, "bottom_servo");
        ColorSensorArm =  hardwareMap.get (Servo.class, "cs_servo");
        ColorSensor =hardwareMap.get(ColorSensor.class,"sensor_color");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Motors labeled if your looking at the FRONT of the robot
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        boolean bLedOn = false;
        TopServo.setPosition(1);
        BottomServo.setPosition(1);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        waitForStart();

        while (opModeIsActive()) {
            if (  ColorSensor.red() <   ColorSensor.blue()) {
                TopServo.setPosition(1);
                BottomServo.setPosition(.5);
                telemetry.addData("Color", "blue");

            }
            if (  ColorSensor.blue() <   ColorSensor.red()) {
                TopServo.setPosition(.5);
                BottomServo.setPosition(1);
                telemetry.addData("Color", "red");
            }


            // Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Right Servo Position",   BottomServo.getPosition());
            telemetry.addData("Left Servo Position",   TopServo.getPosition());
            telemetry.addData("Cs Servo Position",   ColorSensorArm.getPosition());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear",   ColorSensor.alpha());
            telemetry.addData("Red  ",   ColorSensor.red());
            telemetry.addData("Green",   ColorSensor.green());
            telemetry.addData("Blue ",   ColorSensor.blue());
            telemetry.update();
        }

    }
}
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



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

@TeleOp(name="Main6712", group="Linear Opmode")
//@Disabled
public class Main6712TC extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftDriveFront = null;
    private DcMotor RightDriveFront = null;
    private DcMotor LeftDriveBack = null;
    private DcMotor RightDriveBack = null;
    private DcMotor Pulley = null;
    private Servo rightServo = null;
    private Servo leftServo = null;
    private Servo ColorSensor =null;
    int servovalueright = 1;
    int servovalueleft = 1;
    private ColorSensor colorSensor;  // Hardware Device Object


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Motors labeled if your looking at the FRONT of the robot
        LeftDriveFront = hardwareMap.get(DcMotor.class, "left_drive");
        RightDriveFront = hardwareMap.get(DcMotor.class, "right_drive");
        LeftDriveBack = hardwareMap.get(DcMotor.class, "left_drive2");
        RightDriveBack = hardwareMap.get(DcMotor.class, "right_drive2");
        Pulley = hardwareMap.get(DcMotor.class, "pulley");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        ColorSensor = hardwareMap.get(Servo.class, "cs_servo");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        boolean bLedOn = true;


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        RightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        LeftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        RightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        Pulley.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetr
            double leftPower;
            double rightPower;
            double pulleyPower = 0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.left_stick_x;//no negative
            double lift = gamepad1.right_stick_y;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            pulleyPower = Range.clip(lift, -1, 1);
            if(gamepad2.right_bumper) {
                ColorSensor.setPosition(0);
            }
            else if(gamepad2.left_bumper){
            ColorSensor.setPosition(.67);
            }

            if (gamepad1.right_bumper) {
                servovalueright *= (-1);
                while (gamepad1.right_bumper) {
                    servovalueright = servovalueright;
                }
            }
            if (gamepad1.left_bumper) {
                servovalueleft *= (-1);
                while (gamepad1.left_bumper) {
                    servovalueleft = servovalueleft;
                        }
                    }
            if(gamepad1.right_stick_button){
                servovalueright *= (-1);
                servovalueleft *= (-1);
                        while (gamepad1.right_stick_button){
                            servovalueleft = servovalueleft;
                            servovalueright = servovalueright;
                        }
                    }
                    if(gamepad1.dpad_up){Pulley.setTargetPosition((int) 2852.28);
                    }
                    else if(gamepad1.dpad_down){Pulley.setTargetPosition((int) -2852.28);
                    }
//475.38200339559=1 inch
                    if (servovalueright == -1) {
                        rightServo.setPosition(.5);
                    } else if (servovalueright == 1) {
                        rightServo.setPosition(Servo.MAX_POSITION);
                    }
                    if (servovalueleft == -1) {
                        leftServo.setPosition(.5);
                    } else if (servovalueleft == 1) {
                        leftServo.setPosition(Servo.MAX_POSITION);
                    }




                    // Send calculated power to wheels
                    LeftDriveFront.setPower(leftPower);
                    RightDriveFront.setPower(rightPower);
                    LeftDriveBack.setPower(leftPower);
                    RightDriveBack.setPower(rightPower);
                    Pulley.setPower(pulleyPower);

            // Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // Show the elapsed game time and wheel power.
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                    telemetry.addData("Right Servo Position", rightServo.getPosition());
                    telemetry.addData("Left Servo Position", leftServo.getPosition());
                    telemetry.addData("Cs Servo Position", ColorSensor.getPosition());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
                    telemetry.update();
                }

        }
    }

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.lang.annotation.Target;




@TeleOp(name="Main6712", group="Linear Opmode")
//@Disabled
public class Main6712 extends LinearOpMode {
    HardwareMain6712 Robot = new HardwareMain6712();
    HardwareMap hwMap           =  null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Motors labeled if your looking at the FRONT of the robot
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        boolean bLedOn = true;


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        Robot.LeftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        Robot.RightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        Robot.LeftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        Robot.RightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        Robot.Pulley.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        Robot.Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Wait for the game to start (driver presses PLAY)


        waitForStart();
        Robot.runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetr
            double leftPower;
            double rightPower;
            double pulleyPower = 0;
            int targetposition;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.left_stick_x;//no negative
            double lift = gamepad1.right_stick_y;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            pulleyPower = Range.clip(lift, -.5, .5);
            //USED TO CONTROLL COLOR SENSOR ARM IN TELEOP IF NEEDED
            if (gamepad2.right_bumper) {
                Robot.ColorSensorArm.setPosition(0);
            } else if (gamepad2.left_bumper) {
                Robot.ColorSensorArm.setPosition(.67);
            }
            //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE THE TOP
            if (gamepad1.right_bumper) {
                Robot.servovaluetop *= (-1);
                while (gamepad1.right_bumper) {
                    Robot.servovaluetop = Robot.servovaluetop;
                }
            }
            //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE THE BOTTOM

            if (gamepad1.left_bumper) {
                Robot.servovaluebottom *= (-1);
                while (gamepad1.left_bumper) {
                    Robot.servovaluebottom = Robot.servovaluebottom;
                }
            }
            //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE BOTH CLAWS
            if (gamepad1.right_stick_button) {
                Robot.servovaluetop *= (-1);
                Robot.servovaluebottom *= (-1);
                while (gamepad1.right_stick_button) {
                    Robot.servovaluebottom = Robot.servovaluebottom;
                    Robot.servovaluetop = Robot.servovaluetop;
                }
            }
            //LIFT THE MAST UP USING ENCODERS
            if (gamepad1.dpad_up) {
                targetposition = Robot.Pulley.getCurrentPosition() + (6 * Robot.LiftCountsPerInch);
                Robot.Pulley.setTargetPosition(targetposition);
                Robot.Pulley.setPower(1);
                while (gamepad1.dpad_up) {
                    targetposition = targetposition;
                }
            }
            //MOVE THE MAST DOWN USING ENCODERS
            else if (gamepad1.dpad_down) {
                targetposition = Robot.Pulley.getCurrentPosition() - (6 * Robot.LiftCountsPerInch);
                Robot.Pulley.setTargetPosition(targetposition);
                Robot.Pulley.setPower(1);
                while (gamepad1.dpad_down) {
                    targetposition = targetposition;
                }
            }
            //OPEN AMD CLOSE TOP CLAW
            if (Robot.servovaluetop == -1) {
                Robot.TopServo.setPosition(.5);
            } else if (Robot.servovaluetop == 1) {
                Robot.TopServo.setPosition(Servo.MAX_POSITION);
            }
            //OPEN AMD CLOSE BOTTOM CLAW
            if (Robot.servovaluebottom == -1) {
                Robot.BottomServo.setPosition(.5);
            } else if (Robot.servovaluebottom == 1) {
                Robot.BottomServo.setPosition(Servo.MAX_POSITION);
            }


            // Send calculated power to wheels
            Robot.LeftDriveFront.setPower(leftPower);
            Robot.RightDriveFront.setPower(rightPower);
            Robot.LeftDriveBack.setPower(leftPower);
            Robot.RightDriveBack.setPower(rightPower);
            Robot.Pulley.setPower(pulleyPower);

            // Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + Robot.runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Top Servo Position", Robot.TopServo.getPosition());
            telemetry.addData("Bottom Servo Position", Robot.BottomServo.getPosition());
            telemetry.addData("Cs Servo Position", Robot.ColorSensorArm.getPosition());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", Robot.ColorSensor.alpha());
            telemetry.addData("Red  ", Robot.ColorSensor.red());
            telemetry.addData("Green", Robot.ColorSensor.green());
            telemetry.addData("Blue ", Robot.ColorSensor.blue());
            telemetry.update();
        }
    }
}


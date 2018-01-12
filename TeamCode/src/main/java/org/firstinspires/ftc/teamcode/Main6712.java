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
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.lang.annotation.Target;




@TeleOp(name="Main6712", group="Linear Opmode")
//@Disabled
public class Main6712 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public ColorSensor  ColorSensor;
    public DcMotor      LeftDriveFront = null;
    public DcMotor      RightDriveFront = null;
    public DcMotor      LeftDriveBack = null;
    public DcMotor      RightDriveBack = null;
    public DcMotor      Pulley = null;
    public DcMotor      RelicArm =  null;
    public Servo        TopServo = null;
    public Servo        BottomServo = null;
    public Servo        ColorSensorArm =null;
    public Servo        RelicServo = null;
    public Servo        RelicServoClaw = null;
    public int          servovaluetop = 1;
    public int          servovaluebottom = 1;
    public int          servovaluerelic1 = 1;
    public int          servovaluerelic2 = 1;
    public int          LiftCountsPerInch = 475;
    public int          LiftTargetPosition = 0;


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
        RelicArm = hardwareMap.get(DcMotor.class, "relic_arm");
        RelicServo =  hardwareMap.get (Servo.class, "relic_servo");
        RelicServoClaw =  hardwareMap.get (Servo.class, "relic_servo");
        LeftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        RightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        LeftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        RightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        Pulley.setDirection(DcMotor.Direction.REVERSE);
        RelicArm.setDirection(DcMotor.Direction.REVERSE);
       // Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        //Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Wait for the game to start (driver presses PLAY)


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetr
            double leftPower;
            double rightPower;
            double pulleyPower;
            double relicPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive  = gamepad1.left_stick_y;
            double turn   = -gamepad1.left_stick_x;//no negative
            double lift = gamepad1.right_stick_y;
            double relic = gamepad2.right_stick_y;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            pulleyPower = Range.clip(lift, -1, 1);
            relicPower = Range.clip(relic,-1,1);
            //USED TO CONTROLL COLOR SENSOR ARM IN TELEOP IF NEEDED
            if (gamepad2.a) {
                ColorSensorArm.setPosition(0);
            }
            if (gamepad2.b){
                ColorSensorArm.setPosition(.8);
            }

            if(gamepad2.right_bumper){
                servovaluerelic1*= (-1);
                while (gamepad2.right_bumper){
                    servovaluerelic1 = servovaluerelic1;
                }
            }
            if(gamepad2.left_bumper){
                servovaluerelic2*= (-1);
                while (gamepad2.right_bumper){
                    servovaluerelic2 = servovaluerelic2;
                }
            }

            //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE THE TOP
            if (gamepad1.right_bumper) {
                servovaluetop *= (-1);
                while (gamepad1.right_bumper) {
                    servovaluetop = servovaluetop;
                }
            }
            //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE THE BOTTOM

            if (gamepad1.left_bumper) {
                servovaluebottom *= (-1);
                while (gamepad1.left_bumper) {
                    servovaluebottom = servovaluebottom;
                }
            }
            //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE BOTH CLAWS
            if (gamepad1.right_stick_button) {
                servovaluetop *= (-1);
                servovaluebottom *= (-1);
                while (gamepad1.right_stick_button) {
                    servovaluebottom = servovaluebottom;
                    servovaluetop = servovaluetop;
                }
            }
            //LIFT THE MAST UP USING ENCODERS
          /* if (gamepad1.dpad_up) {
                LiftTargetPosition = Pulley.getCurrentPosition() + (6 * LiftCountsPerInch);
                Pulley.setTargetPosition(LiftTargetPosition);
                Pulley.setPower(1);

            }
            //MOVE THE MAST DOWN USING ENCODERS
            else if (gamepad1.dpad_down) {
                LiftTargetPosition = Pulley.getCurrentPosition() - (6 * LiftCountsPerInch);
                Pulley.setTargetPosition(LiftTargetPosition);
                Pulley.setPower(1);
            }*/
            //OPEN AMD CLOSE TOP CLAW
            if (servovaluetop == -1) {
                TopServo.setPosition(0);
            } else if (servovaluetop == 1) {
                TopServo.setPosition(Servo.MAX_POSITION);
            }
            //OPEN AMD CLOSE BOTTOM CLAW
            if (servovaluebottom == -1) {
                BottomServo.setPosition(.5);
            } else if (servovaluebottom == 1) {
                BottomServo.setPosition(Servo.MAX_POSITION);
            }
            /*if (servovaluerelic1 == -1) {
                .setPosition(0);
            } else if (servovaluebottom == 1) {
                BottomServo.setPosition(.5);
            }
            if (servovaluerelic2 == -1) {
                BottomServo.setPosition(0);
            } else if (servovaluebottom == 1) {
                BottomServo.setPosition(1);
            }*/




            // Send calculated power to wheels
            LeftDriveFront.setPower(leftPower);
            RightDriveFront.setPower(rightPower);
            LeftDriveBack.setPower(leftPower);
            RightDriveBack.setPower(rightPower);
            Pulley.setPower(pulleyPower);
            RelicArm.setPower(relicPower);

            // Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Top Servo Position", TopServo.getPosition());
            telemetry.addData("Bottom Servo Position", BottomServo.getPosition());
            telemetry.addData("Cs Servo Position", ColorSensorArm.getPosition());
            telemetry.addData("pulley_position",Pulley.getCurrentPosition());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", ColorSensor.alpha());
            telemetry.addData("Red  ", ColorSensor.red());
            telemetry.addData("Green", ColorSensor.green());
            telemetry.addData("Blue ", ColorSensor.blue());
            telemetry.update();
        }
    }
}


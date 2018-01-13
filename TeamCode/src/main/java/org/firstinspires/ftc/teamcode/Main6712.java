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
import org.firstinspires.ftc.teamcode.input.Input;
import org.firstinspires.ftc.teamcode.input.InputHandlerThread;

import java.lang.annotation.Target;




@TeleOp(name="Main6712", group="Linear Opmode")
//@Disabled
public class Main6712 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor  ColorSensor;
    private DcMotor      LeftDriveFront = null;
    private DcMotor      RightDriveFront = null;
    private DcMotor      LeftDriveBack = null;
    private DcMotor      RightDriveBack = null;
    private DcMotor      Pulley = null;
    private DcMotor      RelicArm =  null;
    private Servo        TopServo = null;
    private Servo        BottomServo = null;
    private Servo        ColorSensorArm =null;
    private Servo        RelicServo = null;
    private Servo        RelicServoClaw = null;
    private int          servovaluetop = 1;
    private int          servovaluebottom = 1;
    private int          servovaluerelic1 = 1;
    private int          servovaluerelic2 = 1;
    private int          LiftCountsPerInch = 475;
    private int          LiftTargetPosition = 0;

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

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        //Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        InputHandlerThread handlerG1 = new InputHandlerThread(this,gamepad1);
        InputHandlerThread handlerG2 = new InputHandlerThread(this,gamepad2);
        
        //USED TO CONTROLL COLOR SENSOR ARM IN TELEOP IF NEEDED
        handlerG2.registerListener(Input.Source.A,
                new Input.Listener() {
                    @Override
                    public void input() {
                        ColorSensorArm.setPosition(0);
                    }
                });
        handlerG2.registerListener(Input.Source.B,
                new Input.Listener() {
                    @Override
                    public void input() {
                        ColorSensorArm.setPosition(.85d);
                    }
                });
        handlerG2.registerListener(Input.Source.RIGHT_BUMPER,
                new Input.Listener() {
                    @Override
                    public void input() {
                        servovaluerelic1 *= -1;
                    }
                });
        handlerG2.registerListener(Input.Source.LEFT_BUMPER,
                new Input.Listener() {
                    @Override
                    public void input() {
                        servovaluerelic2 *= -1;
                    }
                });
        //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE THE TOP
        handlerG2.registerListener(Input.Source.LEFT_BUMPER,
                new Input.Listener() {
                    @Override
                    public void input() {
                        ColorSensorArm.setPosition(2d / 3d);
                    }
                });
        //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE THE BOTTOM
        handlerG1.registerListener(Input.Source.RIGHT_BUMPER,
                new Input.Listener() {
                    @Override
                    public void input() {
                        servovaluetop *= -1;
                    }
                });
        handlerG1.registerListener(Input.Source.RIGHT_BUMPER,
                new Input.Listener() {
                    @Override
                    public void input() {
                        servovaluebottom *= -1;
                    }
                });
        //USE ALTERNATING VALUE PATTERN TO OPEN AND CLOSE BOTH CLAWS
        handlerG1.registerListener(Input.Source.RIGHT_STICK_BUTTON,
                new Input.Listener() {
                    @Override
                    public void input() {
                        servovaluetop *= -1;
                        servovaluebottom *= -1;
                    }
                });
        //LIFT THE MAST UP USING ENCODERS
        handlerG1.registerListener(Input.Source.DPAD_UP,
                new Input.Listener() {
                    @Override
                    public void input()
                        {
                            LiftTargetPosition = Pulley.getCurrentPosition() + (6 * LiftCountsPerInch);
                            Pulley.setTargetPosition(LiftTargetPosition);
                            Pulley.setPower(1);
                        }

                } );
        //MOVE THE MAST DOWN USING ENCODERS
        handlerG1.registerListener(Input.Source.DPAD_DOWN,
                new Input.Listener() {
                    @Override
                    public void input(){
                        LiftTargetPosition = Pulley.getCurrentPosition() - (6 * LiftCountsPerInch);
                        Pulley.setTargetPosition(LiftTargetPosition);
                        Pulley.setPower(1);
                    }
                });

        /*contains actions that are executed every iteration
        (setting the wheel and pulley power, telemetry).*/
        handlerG2.addIterationRunnable(new Runnable() {
            @Override
            public void run() {
                // Choose to drive using either Tank Mode, or POV Mode
                // Comment out the method that's not used.  The default below is POV.

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = gamepad1.left_stick_y;
                double turn = -gamepad1.left_stick_x;//no negative
                double lift = gamepad1.right_stick_y;
                double relic = gamepad2.right_stick_y;
                // Setup a variable for each drive wheel to save power level for telemetry
                double leftPower = Range.clip(drive + turn, -1.0, 1.0);
                double rightPower = Range.clip(drive - turn, -1.0, 1.0);
                double pulleyPower = Range.clip(lift, -1, 1);
                double relicPower = Range.clip(relic,-1,1);

                //OPEN AMD CLOSE TOP CLAW
                TopServo.setPosition((servovaluetop == -1) ? 0.5d : Servo.MAX_POSITION);

                //OPEN AMD CLOSE BOTTOM CLAW
                BottomServo.setPosition((servovaluebottom == -1) ? 0.5d : Servo.MAX_POSITION);

                RelicServo.setPosition(servovaluerelic1 == -1 ? .5d : Servo.MAX_POSITION);

                RelicServoClaw.setPosition(servovaluerelic2 == -1 ? 0 : .5d);

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
                //telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", ColorSensor.alpha());
                telemetry.addData("Red  ", ColorSensor.red());
                telemetry.addData("Green", ColorSensor.green());
                telemetry.addData("Blue ", ColorSensor.blue());
                telemetry.update();
            }
        });

        handlerG1.start();
        handlerG2.start();

        runtime.reset();
    }
}


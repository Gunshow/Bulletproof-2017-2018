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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareMain6712
{
    /* Public OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();
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
    public int     servovaluetop = 1;
    public int     servovaluebottom = 1;
    public int     LiftCountsPerInch = 475;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     Lift_DIAMETER_INCHES    = .75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_INCH_Lift    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (Lift_DIAMETER_INCHES * 3.1415) ;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    float hsvValues[] = {0F, 0F, 0F};
    public ColorSensor ColorSensor;  // Hardware Device Object

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMain6712(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LeftDriveFront = hwMap.get(DcMotor.class, "left_drive");
        RightDriveFront = hwMap.get(DcMotor.class, "right_drive");
        LeftDriveBack = hwMap.get(DcMotor.class, "left_drive2");
        RightDriveBack = hwMap.get(DcMotor.class, "right_drive2");
        Pulley = hwMap.get(DcMotor.class, "pulley");
        TopServo  = hwMap.get(Servo.class, "top_servo");
        BottomServo = hwMap.get(Servo.class, "bottom_servo");
        ColorSensorArm = hwMap.get(Servo.class, "cs_servo");
        ColorSensor = hwMap.colorSensor.get("sensor_color");
        LeftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        RightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        LeftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        RightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        Pulley.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        LeftDriveFront.setPower(0);
        RightDriveFront.setPower(0);
        LeftDriveBack.setPower(0);
        RightDriveBack.setPower(0);
        Pulley.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // Define and initialize ALL installed servos.

        TopServo.setPosition(Servo.MAX_POSITION);
        BottomServo.setPosition(Servo.MAX_POSITION);
    }
 }


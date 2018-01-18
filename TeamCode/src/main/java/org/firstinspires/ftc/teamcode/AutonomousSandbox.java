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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutiSandBox", group="Pushbot")
//@Disabled
public class AutonomousSandbox extends LinearOpMode {


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
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
    private int     servovaluetop = 1;
    private int     servovaluebottom = 1;
    private int     LiftCountsPerInch = 475;
    private ColorSensor ColorSensor = null;
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
    VuforiaLocalizer vuforia;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables visionTargets;
    VuforiaTrackable   Target;
    private VuforiaTrackableDefaultListener listener;
    private String pathV = null;
    private String pathCS = null;
    private String path =pathV + pathCS ;
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        LeftDriveFront = hardwareMap.get(DcMotor.class, "left_drive");
        RightDriveFront = hardwareMap.get(DcMotor.class, "right_drive");
        LeftDriveBack = hardwareMap.get(DcMotor.class, "left_drive2");
        RightDriveBack = hardwareMap.get(DcMotor.class, "right_drive2");
        Pulley = hardwareMap.get(DcMotor.class, "pulley");
        TopServo = hardwareMap.get(Servo.class, "top_servo");
        BottomServo = hardwareMap.get(Servo.class, "bottom_servo");
        ColorSensorArm = hardwareMap.get(Servo.class, "cs_servo");
        ColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        LeftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        RightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        LeftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        RightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        Pulley.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQHpBJD/////AAAAGZVP9yMZskCVuGhdevkuv5Fj9ijFPt0+ZBjva+5acSsF7qURU3zIDv7VM1RI1iNmDa2PsGjl8xvFcqUWMHDz/oVmDY9HirmEqalP8zk4mK1PO6KQ0EKjo7n+dYatfwnheThOUKbWYYRkYLZa+QWDvyFbLip9P+VwLHy2FKj66zY2wwmIk6Y44gyAl3SCseIyfMqYLP508iFhi5V7CZCH20GB2jtcdQEgJxl7dNjHQDidQTidgVvRpxR64eovG8ASKJClQMhZivfrBE3PzbjLj5K7nF1NVwx/GEedHsY24cKxhjL3ci0O3MKZ5ih4EgZARBGTUnLKwJWyBI4mcSqrukvPzbPimoz+3S9v6nFdblJR";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        visionTargets = this.vuforia.loadTrackablesFromAsset("Bp_OT");
        Target = visionTargets.get(0);
        Target.setName("Calc");
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                LeftDriveFront.getCurrentPosition(),
                RightDriveFront.getCurrentPosition(),
                LeftDriveBack.getCurrentPosition(),
                RightDriveBack.getCurrentPosition(),
                Pulley.getCurrentPosition());
        TopServo.setPosition(1);
        BottomServo.setPosition(1);


        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        runtime.reset();
        relicTrackables.activate();
        visionTargets.activate();

        telemetry.addData("Tracking " + Target.getName(), listener.isVisible());
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        waitForStart();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        listener = (VuforiaTrackableDefaultListener) Target.getListener();
        TopServo.setPosition(1);
        BottomServo.setPosition(1);
        sleep(1000);                                                 //S0:Wait 5 seconds to start
        BottomServo.setPosition(.5);                                       // S1:Grab cube
        TopServo.setPosition(.5);
        sleep(1000);
        ColorSensorArm.setPosition(.85);
        sleep(1000);
       /* if (ColorSensor.red() < ColorSensor.blue()) {
            pathCS = "B";
            telemetry.addData("Color", "blue");
        } else if (ColorSensor.blue() < ColorSensor.red()) {
            pathCS = "R";
            telemetry.addData("Color", "red");
        }*/
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            telemetry.addData("Tracking " + Target.getName(), listener.isVisible());


            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                pathV = "R";
                telemetry.addData("vumark","Right");

            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                pathV = "C";
                telemetry.addData("vumark","Center");

            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                pathV = "L";
                telemetry.addData("vumark","Left");

            } else {
                path = path;
            }

            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
        }
        /*if (path == "RR") {
            telemetry.addData("Path", path);
        } else if (path == "CR") {
            telemetry.addData("Path", path);

        } else if (path == "LR") {
            telemetry.addData("Path", path);

        } else if (path == "RB") {
            telemetry.addData("Path", path);

        } else if (path == "CB") {
            telemetry.addData("Path", path);

        } else if (path == "LB") {
            telemetry.addData("Path", path);
        }*/

        sleep(10000);
        telemetry.addData("Path", "Complete");
        telemetry.addData("Path", path);
        telemetry.update();
    }
    //1 lift inch is = 6.25 inchs '
    // timeout and lift are backwards
    private void encoderDrive(double speed,
                              double leftInches,
                              double rightInches,
                              // double liftInches,
                              double timeoutS) {

        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;
        //   int newLiftTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftTarget = LeftDriveFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = RightDriveFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget2 = LeftDriveBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget2 = RightDriveBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            //newLiftTarget = Pulley.getCurrentPosition()  + (int)(liftInches * COUNTS_PER_INCH_Lift);
            LeftDriveFront.setTargetPosition(newLeftTarget);
            RightDriveFront.setTargetPosition(newRightTarget);
            LeftDriveBack.setTargetPosition(newLeftTarget2);
            RightDriveBack.setTargetPosition(newRightTarget2);
            //    Pulley.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            LeftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftDriveFront.setPower(Math.abs(speed));
            RightDriveFront.setPower(Math.abs(speed));
            LeftDriveBack.setPower(Math.abs(speed));
            RightDriveBack.setPower(Math.abs(speed));
            //Pulley.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftDriveFront.isBusy() && RightDriveFront.isBusy()// && //Pulley.isBusy()
                            && LeftDriveBack.isBusy() && RightDriveBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget, newLeftTarget2,  newRightTarget2);//newLiftTarget
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        //Pulley.getCurrentPosition(),
                        LeftDriveFront.getCurrentPosition(),
                        RightDriveFront.getCurrentPosition(),
                        LeftDriveBack.getCurrentPosition(),
                        RightDriveBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftDriveFront.setPower(0);
            RightDriveFront.setPower(0);
            LeftDriveBack.setPower(0);
            RightDriveBack.setPower(0);
            //  Pulley.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }
}

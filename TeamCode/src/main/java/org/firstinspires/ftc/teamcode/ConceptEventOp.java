package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ObjectTarget;
import com.vuforia.VuMarkTemplate;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.lang.annotation.Target;

@Autonomous(name = "Concept: NullOp", group = "Concept")
//@Disabled
public class ConceptEventOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private ColorSensor ColorSensor;
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
    private int          servovaluetop = 1;
    private int          servovaluebottom = 1;
    private int          servovaluerelic1 = 1;
    private int          servovaluerelic2 = 1;
    private int          LiftCountsPerInch = 475;
    private int          LiftTargetPosition = 0;
    //vuforia initialization
    VuforiaLocalizer vuforia;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables visionTargets;
    VuforiaTrackable   Target;
    private VuforiaTrackableDefaultListener listener;
    private char pathV = 'N';
    private char pathCS ='N';
    private String path = ("pathV" + "pathCS");

    @Override
    public void runOpMode() {
        LeftDriveFront = hardwareMap.get(DcMotor.class, "left_drive");
        RightDriveFront = hardwareMap.get(DcMotor.class, "right_drive");
        LeftDriveBack = hardwareMap.get(DcMotor.class, "left_drive2");
        RightDriveBack =  hardwareMap.get (DcMotor.class, "right_drive2");
        Pulley =  hardwareMap.get (DcMotor.class, "pulley");
        TopServo  =  hardwareMap.get (Servo.class, "top_servo");
        BottomServo =  hardwareMap.get (Servo.class, "bottom_servo");
        ColorSensorArm =  hardwareMap.get (Servo.class, "cs_servo");
        ColorSensor =hardwareMap.get(ColorSensor.class,"sensor_color");
        RightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        LeftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        RightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQHpBJD/////AAAAGZVP9yMZskCVuGhdevkuv5Fj9ijFPt0+ZBjva+5acSsF7qURU3zIDv7VM1RI1iNmDa2PsGjl8xvFcqUWMHDz/oVmDY9HirmEqalP8zk4mK1PO6KQ0EKjo7n+dYatfwnheThOUKbWYYRkYLZa+QWDvyFbLip9P+VwLHy2FKj66zY2wwmIk6Y44gyAl3SCseIyfMqYLP508iFhi5V7CZCH20GB2jtcdQEgJxl7dNjHQDidQTidgVvRpxR64eovG8ASKJClQMhZivfrBE3PzbjLj5K7nF1NVwx/GEedHsY24cKxhjL3ci0O3MKZ5ih4EgZARBGTUnLKwJWyBI4mcSqrukvPzbPimoz+3S9v6nFdblJR";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        visionTargets = this.vuforia.loadTrackablesFromAsset("Bp_OT");
        Target =  visionTargets.get(0);
        Target.setName("Calc");
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary





        runtime.reset();
        relicTrackables.activate();
        visionTargets.activate();



        waitForStart();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        listener = (VuforiaTrackableDefaultListener) Target.getListener();
        telemetry.addData("Tracking " + Target.getName(), listener.isVisible());
    /*    if(listener.isVisible()){
            TopServo.setPosition(1);
            BottomServo.setPosition(.525);
        }
        else if(!(listener.isVisible())){
            TopServo.setPosition(.18);
            BottomServo.setPosition(1);
        }
        else{TopServo.setPosition(1);
            BottomServo.setPosition(1);
        }*/
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

            telemetry.addData("Tracking " + Target.getName(), listener.isVisible());

            if (vuMark == RelicRecoveryVuMark.RIGHT){
                encoderDrive(DRIVE_SPEED,6,6,3);
            telemetry.addData("VuMark", "Right");}

            else if (vuMark == RelicRecoveryVuMark.CENTER) {
                encoderDrive(DRIVE_SPEED,6,-6,3);
                telemetry.addData("VuMark", "Center");}

            else if (vuMark == RelicRecoveryVuMark.LEFT){
                encoderDrive(DRIVE_SPEED,-6,-6,3);
                telemetry.addData("VuMark", "Left");
            }

            else if (vuMark == RelicRecoveryVuMark.UNKNOWN){
                telemetry.addData("VuMark", "Unknown");}

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
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.update();
    }
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
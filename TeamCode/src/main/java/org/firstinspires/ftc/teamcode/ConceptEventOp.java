package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
public class ConceptEventOp extends OpMode {

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
    public void init() {
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

    }

    @Override
    public void start() {

        runtime.reset();
        relicTrackables.activate();
        visionTargets.activate();

    }

    @Override
    public void loop() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        listener = (VuforiaTrackableDefaultListener) Target.getListener();
        telemetry.addData("Tracking " + Target.getName(), listener.isVisible());
        if(listener.isVisible()){
            TopServo.setPosition(1);
            BottomServo.setPosition(.525);
        }
        else if(!(listener.isVisible())){
            TopServo.setPosition(.18);
            BottomServo.setPosition(1);
        }
        else{TopServo.setPosition(1);
            BottomServo.setPosition(1);
        }
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

            telemetry.addData("Tracking " + Target.getName(), listener.isVisible());

            if (vuMark == RelicRecoveryVuMark.RIGHT){
                /* if(listener.isVisible()){
                     LeftDriveBack.setPower(0);
                     RightDriveBack.setPower(0);
                     LeftDriveFront.setPower(0);
                     RightDriveFront.setPower(0);
                 }
                else if(!listener.isVisible()){
                    LeftDriveBack.setPower(-.1);
                    RightDriveBack.setPower(-.1);
                    LeftDriveFront.setPower(-.1);
                    RightDriveFront.setPower(-.1);
                }*/
            telemetry.addData("VuMark", "Right");}
            else if (vuMark == RelicRecoveryVuMark.CENTER)
            {
                LeftDriveBack.setPower(0);
                RightDriveBack.setPower(0);
                LeftDriveFront.setPower(0);
                RightDriveFront.setPower(0);
                telemetry.addData("VuMark", "Center");}
            else if (vuMark == RelicRecoveryVuMark.LEFT){
                LeftDriveBack.setPower(-.1);
                RightDriveBack.setPower(-.1);
                LeftDriveFront.setPower(-.1);
                RightDriveFront.setPower(-.1);
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

}
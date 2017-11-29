package org.firstinspires.ftc.teamcode;

/**
 * Created by brybr on 11/4/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name= "Servo Test",group= "linearOpMode")
//@Disabled
public class ServoTest extends Main6712TC{
    private ElapsedTime runtime = new ElapsedTime();
    private Servo rightServo = null;
    private Servo leftServo = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        rightServo = hardwareMap.get(Servo.class,"right_servo");
        leftServo  = hardwareMap.get(Servo.class,  "left_servo");
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            // run until the end of the match (driver presses STOP)
                // check to see if we need to move the servo.
                if(gamepad1.right_bumper) {
                    // move to 0 degrees.
                    rightServo.setPosition(0);
                    leftServo.setPosition(1);
                } else if (gamepad1.left_bumper) {
                    // move to 90 degrees.
                    rightServo.setPosition(0.5);
                    leftServo.setPosition(0.5);
                } else if (gamepad1.y) {
                    // move to 180 degrees.
                    rightServo.setPosition(1);
                    leftServo.setPosition(0);
                }
                telemetry.addData("Right Servo Position", rightServo.getPosition());
                telemetry.addData("Left Servo Position", leftServo.getPosition());
                telemetry.addData("Status", "Running");
                telemetry.update();

            }
        }
    }
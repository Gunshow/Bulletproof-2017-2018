package org.firstinspires.ftc.teamcode;

/**
 * Created by brybr on 11/4/2017.
 */

import android.util.Log;
import android.view.View;
import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name= "Servo Test",group= "linearOpMode")
//@Disabled
public class ServoTest extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private Servo rightServo = null;
    private Servo leftServo = null;
    int servovalue = 1;
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
           if(gamepad1.a){
               servovalue *= (-1);
               while(gamepad1.a) {
                   servovalue = servovalue;
               }
           }
           // else if (gamepad1.b){
              // servovalue= (-1);
           //}
           if(servovalue == -1){
            rightServo.setPosition(.2);
            leftServo.setPosition(.8);}
            else if (servovalue == 1){
            rightServo.setPosition(.5);
            leftServo.setPosition(.5);}

                }
                telemetry.addData("Right Servo Position", rightServo.getPosition());
                telemetry.addData("Left Servo Position", leftServo.getPosition());
                telemetry.addData("Status", "Running");
                telemetry.update();

            }
        }
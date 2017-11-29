package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by brybr on 11/28/2017.
 */
@TeleOp(name = "Lift with encoders",group = "Linear OpMode")
public class encoderLiftTest extends LinearOpMode

{
//@Disabled

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor leftDrive2  = null;
    private DcMotor rightDrive2 = null;
    private DcMotor pulley      = null;
    private Servo rightServo  = null;
    private Servo   leftServo   = null;

}

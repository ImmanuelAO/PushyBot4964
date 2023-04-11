package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="teleOP!!!!!!!!")
public class teleOP extends OpMode {


    public static DcMotor LeftDT  = null;
    public static DcMotor RightDT = null;
    public static DcMotor Lift     = null;
    public static DcMotor Claw     = null;


    public double speedMode = 1;
    public boolean xIsHeld = false;
    public boolean bIsHeld = false;
    public boolean dpadLeftIsHeld = false;
    public boolean dpadRightIsHeld = false;
    public double spd = 0.4;





    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();


    }


    @Override
    public void loop() {

        //Slow Mode Code for a and b keys
        if (gamepad1.a) {
            speedMode = .6;
        }
        if (gamepad1.b) {
            speedMode = 1;
        }
        //Slow Mode Code for a and b keys




        //Slow Mode Code for bumpers
        if (gamepad1.right_bumper && speedMode > .2) {
            speedMode -= .05;
        } else if (gamepad1.right_trigger >= .5 && speedMode < 2) {
            speedMode += .05;
        }
        //Slow Mode Code for bumpers


        double stopBuffer = 0; //Not currently Implemented



        //Drive Train Code
        double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
        double turn = speedMode * Math.pow(gamepad1.left_stick_x, 3);

        double leftFrontPower = forward + turn;
        double rightFrontPower = forward - turn;
        double[] powers = {leftFrontPower, rightFrontPower};



        LeftDT.setPower(leftFrontPower);

        RightDT.setPower(rightFrontPower);

        //Drive Train Code


        //lift code





        telemetry.addLine(" left encoder counts: " + LeftDT.getCurrentPosition());
        telemetry.addLine(" right encoder counts: " + RightDT.getCurrentPosition());


    }
}
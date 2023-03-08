package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="teleOP!!!!!!!!")
public class teleOP extends OpMode {


    public static DcMotor tLeftDT  = null;
    public static DcMotor bLeftDT  = null;
    public static DcMotor tRightDT = null;
    public static DcMotor bRightDT = null;
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
        double right = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
        double turn = -speedMode * Math.pow(gamepad1.right_stick_x,3);

        double leftFrontPower = forward + right + turn;
        double leftBackPower = forward - right + turn;
        double rightFrontPower = forward - right - turn;
        double rightBackPower = forward + right - turn;
        double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        boolean needToScale = false;
        for (double power : powers){
            if(Math.abs(power) > 1){
                needToScale = true;
                break;
            }
        }
        if (needToScale){
            double greatest = 0;
            for (double power : powers){
                if (Math.abs(power) > greatest){
                    greatest = Math.abs(power);
                }
            }
            leftFrontPower /= greatest;
            leftBackPower /= greatest;
            rightFrontPower /= greatest;
            rightBackPower /= greatest;
        }

        boolean stop = true;
        for (double power : powers){
            if (Math.abs(power) > stopBuffer){
                stop = false;
                break;
            }
        }
        if (stop){
            leftFrontPower = 0;
            leftBackPower = 0;
            rightFrontPower = 0;
            rightBackPower = 0;
        }

        tLeftDT.setPower(leftFrontPower);
        bLeftDT.setPower(leftBackPower);
        tRightDT.setPower(rightFrontPower);
        bRightDT.setPower(rightBackPower);
        //Drive Train Code


        //lift code
        Lift.setPower(4*gamepad2.left_stick_y);



        //wormhole in
        if(gamepad2.y){
            spd += 0.1;
        }
        if(gamepad2.a){
            spd -= 0.1;
        }

        if (gamepad2.b) {
            Claw.setPower(spd);

        }else if (gamepad2.x) {
            Claw.setPower(-spd);

        }else {
            Claw.setPower(0);

        }
        telemetry.addLine("Front left encoder counts: " + tLeftDT.getCurrentPosition());
        telemetry.addLine("Back left encoder counts: " + bLeftDT.getCurrentPosition());
        telemetry.addLine("Front right encoder counts: " + tRightDT.getCurrentPosition());
        telemetry.addLine("Back right encoder counts: " + bRightDT.getCurrentPosition());
        telemetry.addLine("Lift encoder counts: " + Lift.getCurrentPosition());
        telemetry.addLine("Claw encoder counts: " + Claw.getCurrentPosition());

    }
}
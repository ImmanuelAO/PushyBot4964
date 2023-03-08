package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Bot {
    public static DcMotor tLeftDT = null;
    public static DcMotor bLeftDT = null;
    public static DcMotor tRightDT = null;
    public static DcMotor bRightDT = null;
    public static DcMotor Lift = null;
    public static DcMotor Claw = null;

    public static ModernRoboticsI2cGyro Gyro;
    public static Rev2mDistanceSensor distance;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable

    public static double DRIVE_SPEED = 0.4;
    public static double TURN_SPEED = 0.4;
    public static double amountError = 2;


    public static final double conversion = Variables.conversion; // var.conversion of encoder rotations to centimetres
    public static final double tileConversion = conversion * 60; // var.conversion of encoder rotations to tiles !!EDIT!!

    public void init(HardwareMap ahwMap, OpMode opMode) {
        opMode.telemetry.addLine("wait for it... ");
        opMode.telemetry.update();

        HardwareMap hwMap = ahwMap;
        Variables var = new Variables();
        tLeftDT = hwMap.get(DcMotor.class, "FrontL");
        bLeftDT = hwMap.get(DcMotor.class, "BackL");
        tRightDT = hwMap.get(DcMotor.class, "FrontR");
        bRightDT = hwMap.get(DcMotor.class, "BackR");
        Lift = hwMap.get(DcMotor.class, "lift");
        Claw = hwMap.get(DcMotor.class, "claw");
        distance = hwMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        Gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

        tLeftDT.setDirection(DcMotor.Direction.FORWARD);
        tLeftDT.setDirection(DcMotor.Direction.FORWARD);
        bRightDT.setDirection(DcMotor.Direction.REVERSE);
        bLeftDT.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Claw.setDirection(DcMotor.Direction.FORWARD);


        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Gyro.calibrate();

        tRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tRightDT.setPower(0);
        tLeftDT.setPower(0);
        bRightDT.setPower(0);
        bLeftDT.setPower(0);
        Lift.setPower(0);
        Claw.setPower(0);


        opMode.telemetry.addLine("Initialization Complete! ;) ");
        opMode.telemetry.update();

    }

    public static void liftSet(int height, double speed, LinearOpMode opMode) {
        boolean done = false;

        Lift.setTargetPosition((int) (height * 1.1));

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        Lift.setPower(speed);

        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }


    //driving using only Mecanum strafe
    public static void strafeDrive(float distance, double speed, LinearOpMode opMode) {
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if (opMode.opModeIsActive()) {
            double error = distance / Math.abs(distance);

            boolean done = false;

            int tLeftPower = tLeftDT.getCurrentPosition() + (int) (conversion * -distance * 1.1 - (error * 1.5 * speed));
            int bLeftPower = bLeftDT.getCurrentPosition() + (int) (conversion * distance * 1.1 - (error * 1.5 * speed));
            int tRightPower = tRightDT.getCurrentPosition() + (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int bRightPower = bRightDT.getCurrentPosition() + (int) (conversion * -distance * 1.1 + (error * 1.5 * speed));

            tLeftDT.setTargetPosition(tLeftPower);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setPower(speed);
            tRightDT.setPower(speed);
            bLeftDT.setPower(speed);
            bRightDT.setPower(speed);

            while (opMode.opModeIsActive() && !done) {

                double actError = error * (Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition())) + error * (Math.abs(bLeftPower) - Math.abs(bLeftDT.getCurrentPosition())) + error *
                        (Math.abs(tRightPower) - Math.abs(tRightDT.getCurrentPosition())) + error * (Math.abs(bRightPower) - Math.abs(bRightDT.getCurrentPosition()));

                if (error >= actError - 2 && error <= actError + 2) {
                    done = true;
                    tLeftDT.setPower(0);
                    tRightDT.setPower(0);
                    bLeftDT.setPower(0);
                    bRightDT.setPower(0);
                }
            }


        }

    }

    public static void SensorStrafeDrive(float distance, double speed, LinearOpMode opMode) {
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if (opMode.opModeIsActive()) {
            double error = distance / Math.abs(distance);

            boolean done = false;

            int tLeftPower = tLeftDT.getCurrentPosition() + (int) (conversion * -distance * 1.1 - (error * 1.5 * speed));
            int bLeftPower = bLeftDT.getCurrentPosition() + (int) (conversion * distance * 1.1 - (error * 1.5 * speed));
            int tRightPower = tRightDT.getCurrentPosition() + (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int bRightPower = bRightDT.getCurrentPosition() + (int) (conversion * -distance * 1.1 + (error * 1.5 * speed));

            tLeftDT.setTargetPosition(tLeftPower);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setPower(speed);
            tRightDT.setPower(speed);
            bLeftDT.setPower(speed);
            bRightDT.setPower(speed);

            while (opMode.opModeIsActive() && !done) {

                double actError = error * (Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition())) + error * (Math.abs(bLeftPower) - Math.abs(bLeftDT.getCurrentPosition())) + error *
                        (Math.abs(tRightPower) - Math.abs(tRightDT.getCurrentPosition())) + error * (Math.abs(bRightPower) - Math.abs(bRightDT.getCurrentPosition()));
                double currentDistance = Bot.distance.getDistance(DistanceUnit.CM);

                if (error >= actError - 2 && error <= actError + 2 || currentDistance < 30) {
                    done = true;
                    tLeftDT.setPower(0);
                    tRightDT.setPower(0);
                    bLeftDT.setPower(0);
                    bRightDT.setPower(0);
                }

                opMode.telemetry.addLine("distance:" + Bot.distance.getDistance(DistanceUnit.CM));
                opMode.telemetry.update();
            }


        }

    }

    public static void driveStraight(float distance, double speed, LinearOpMode opMode) {
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if (opMode.opModeIsActive()) {
            double error = distance / Math.abs(distance);


            boolean done = false;

            int tLeftPower = tLeftDT.getCurrentPosition() - (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int bLeftPower = bLeftDT.getCurrentPosition() - (int) (conversion * distance * 1.1 + (error * 1.5 * speed));
            int tRightPower = tRightDT.getCurrentPosition() - (int) (conversion * distance * 1.1 - (error * 1.5 * speed));
            int bRightPower = bRightDT.getCurrentPosition() - (int) (conversion * distance * 1.1 - (error * 1.5 * speed));

            tLeftDT.setTargetPosition(tLeftPower);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setPower(speed);
            tRightDT.setPower(speed);
            bLeftDT.setPower(speed);
            bRightDT.setPower(speed);

            while (opMode.opModeIsActive() && !done) {

                double actError = error * (Math.abs(tLeftPower) - Math.abs(tLeftDT.getCurrentPosition())) + error * (Math.abs(bLeftPower) - Math.abs(bLeftDT.getCurrentPosition())) + error *
                        (Math.abs(tRightPower) - Math.abs(tRightDT.getCurrentPosition())) + error * (Math.abs(bRightPower) - Math.abs(bRightDT.getCurrentPosition()));

                if (error >= actError - 2 && error <= actError + 2) {
                    done = true;
                    tLeftDT.setPower(0);
                    tRightDT.setPower(0);
                    bLeftDT.setPower(0);
                    bRightDT.setPower(0);
                }
            }


        }

    }
}
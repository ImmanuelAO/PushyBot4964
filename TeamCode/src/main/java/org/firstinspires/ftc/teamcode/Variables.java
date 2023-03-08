package org.firstinspires.ftc.teamcode;

public class Variables  {
    public int Lvl_Ground = -100,
            Lvl_Short  = -1000,
            Lvl_Mid    = -2000,
            Lvl_Tall   = -3000;
    public boolean slo,
            btnlock = false;
    public int claw_zero = 15, //35,
            claw_cone = -28;
    static final double     COUNTS_PER_MOTOR_REV    = 19;
    static final double     DRIVE_GEAR_REDUCTION    = 40;
    static final double     WHEEL_DIAMETER_INCHES   = 4.125;
    static final double     EMPIRICAL_MULTIPLIER    = (30.0 / 17);
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * EMPIRICAL_MULTIPLIER)
            / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double conversion = 16.7;
}
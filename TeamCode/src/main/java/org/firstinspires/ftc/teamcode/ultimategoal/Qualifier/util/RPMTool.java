package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


/*
 * RPMTool can read and write RPM values
 *
 */
@Config
public class RPMTool {

    public double TICKS_PER_REVOLUTION = 0;

    private ElapsedTime time;

    private DcMotorEx motor;

    private double Time = 0.001; // set a non 0 value to prevent any initial div by 0
    private double ticks = 0;

    private double lastTicks = 0;
    private double lastTime = 0;

    public static double p = 25;
    public static double i = 0;
    public static double d = 6;
    public static double f = 13.2481;


    /*
     * motor that you want to read the RPM of or write the RPM needs to be passed as the motor parameter.
     * The ticks per revolution also needs to be passed as a parameter (you'll find value on motor website).
     */

    public RPMTool(DcMotorEx motor, double TICKS_PER_REVOLUTION){

        this.TICKS_PER_REVOLUTION = TICKS_PER_REVOLUTION;

        this.motor = motor;

        time = new ElapsedTime();

        time.reset();


    }


    // keep track ticks per sec
    public double ticksPerSec(){

        if (Time > 1){
            lastTicks = motor.getCurrentPosition();
            lastTime = time.seconds();
        }

        ticks = motor.getCurrentPosition() - lastTicks;
        Time = time.seconds() - lastTime;

        double tickVelocity = ticks / Time;

        return tickVelocity;
    }

    // multiply by 60 then divide ticks vel by total ticks in one rotation to get the rpm
    public double getRPM(){
        double tickspermin = ticksPerSec() * 60;

        double RPM = tickspermin / TICKS_PER_REVOLUTION;

        RPM = Math.ceil(RPM); // we will round up

        return RPM;
    }

    // convert RPM to ticks per sec then set as velocity
    public void setRPM(double targetRPM){
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // I only have to set this once

        // convert rpm to ticks per sec
        double ticksPerSec = targetRPM * TICKS_PER_REVOLUTION / 60;

        // set velocity
        motor.setVelocityPIDFCoefficients(p,i,d,f);

        motor.setVelocity(ticksPerSec);
    }

}
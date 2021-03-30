package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

public class ShooterControlThread implements Runnable{

    private boolean isRunning = true;

    public DcMotor flywheel = null;

    public ElapsedTime runTime = new ElapsedTime();

    public double lastTick = 0;
    public double lastTime = 0;
    public double currentTick = 0;
    public double currentTime = 0;

    public double currentShooterPower = 0;
    public double targetShooterRPM = 0;

    MorganConstants robotConstants;

    public ShooterControlThread(DcMotor motorFlywheel) {
        this.flywheel = motorFlywheel;
    }   //end of constructor

    /**
     * Method: controlShooter()
     *  -   controls the speed of the motors while the program is running
     */
    public void controlShooter() {
        this.currentShooterPower = shooterPower();
        setShooterPower(this.currentShooterPower);
    }   //end of controlShooter()

    /**
     * Method: setShooterPower(double powerLevel)
     *  -   apply the shooter power to the shooter motor(s)
     */
    private void setShooterPower(double powerLevel) {
        this.flywheel.setPower(powerLevel);
    }   //end of setShooterPower(double powerLevel)

    /**
     * Method: setRPMMeasurements()
     *  -   sets the values for lastTick, currentTick, lastTime, currentTime
     */
    public void setRPMMeasurments() {
        double tempValue = 0;
        this.lastTime = this.currentTime;
        this.lastTick = this.currentTick;

        for(int i = 0; i < 2000000; i++) {
            tempValue = tempValue + i;
        }   // end of for(int i = 0; i < 2000000; i++)

        this.currentTick = flywheel.getCurrentPosition();
        this.currentTime = runTime.time();
    }   //end of setRPMMeasurments()

    /**
     *
     */
    public void setTargetShooterRPM(double targetRPM) {
        this.targetShooterRPM = targetRPM;
    }   //end of setTargetShooterRPM(double targetRPM)

    /**
     * Method: measureRPM()
     *  -   measures the current RPM of the motors
     */
    public double  measureRPM() {
        setRPMMeasurments();
        double rPM = (((Math.abs(this.currentTick - this.lastTick))/this.robotConstants.FLYWHEEL_TICKS_PER_ROTATION)/(Math.abs(this.currentTime - this.lastTime)))*60;

        return rPM;
    }   //end of measureRPM()

    /**
     * Method: stopShooterThread
     * -    stops the shooter thread
     */
    public void stopShooterThread() {
        this.isRunning = false;
    }

    /**
     * Method: shooterPower()
     * -    calculates the power for the flywheel
     */
    public double shooterPower() {
        double shooterPower = this.currentShooterPower;
        double error = this.targetShooterRPM - measureRPM();

        double Cp = robotConstants.SHOOTER_KP; //aka kP
        double Ci = 0.0003; //aka kI
        double Cd = 0.0001; //aka kD

        double maxPower = 1;

        double integral = 0;
        double derivative = 0;

        if(this.targetShooterRPM == 0) {
            shooterPower = 0;
        } else if (this.targetShooterRPM != 0 && shooterPower == 0) {
            shooterPower = 0.50;
        } else {
            shooterPower = shooterPower + ((Cp*error) + (Ci*integral) + (Cd*derivative));
        }   //end of if(this.targetShooterRPM == 0)

        return (Range.clip(shooterPower, 0, maxPower));
    }   //end of shooterPower()

    @Override
    public void run() {
        while(isRunning) {
            controlShooter();
            try {
               Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            } //end of try - catch
        }   //end of while(isRunning)
        setTargetShooterRPM(0);
        controlShooter();
    }   //end of void run()

}   //end of ShooterControlThread
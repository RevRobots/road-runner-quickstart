package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

public class ShooterClass {
    public boolean dualShooter;

    DcMotorEx flywheel1, flywheel2;

    private double currentRPM;
    public double lastTick1 = 0;
    public double lastTick2 = 0;
    public double lastTime = 0;
    public double currentTick1 = 0;
    public double currentTick2 = 0;
    public double currentTime = 0;
    public double RPM = 0;

    MorganConstants robotConstants = new MorganConstants(null);
    ElapsedTime time = new ElapsedTime();

    public ShooterClass (boolean runDualShooter, DcMotorEx flywheelMotor1, DcMotorEx flywheelMotor2) {
        this.dualShooter = runDualShooter;

        this.flywheel1 = flywheelMotor1;
        this.flywheel2 = flywheelMotor2;

        currentRPM = robotConstants.HIGH_GOAL_SHOOTER_RPM;
    }

    public void shooterDriveControl (Gamepad driveGamepad, boolean setVelocity) {
        if(dualShooter == true) {
            if(setVelocity == false) {
                if(driveGamepad.left_trigger != 0) {
                    flywheel1.setPower(1);
                    flywheel2.setPower(1);
                } else {
                    flywheel1.setPower(0.125);
                    flywheel2.setPower(0.125);
                }
            } else if(setVelocity == true) {
                if(driveGamepad.left_trigger != 0) {
                    setFlywheelVelocity(currentRPM, currentRPM);
                } else {
                    setFlywheelVelocity(robotConstants.IDLE_SHOOTER_RPM, robotConstants.IDLE_SHOOTER_RPM);
                }
            }
        } else if(dualShooter == false) {
            if(setVelocity == false) {
                if(driveGamepad.left_trigger != 0) {
                    flywheel1.setPower(1);
                } else {
                    flywheel1.setPower(0.125);
                }
            } else if(setVelocity == true) {
                if(driveGamepad.left_trigger != 0) {
                    setFlywheelVelocity(currentRPM);
                } else {
                    setFlywheelVelocity(robotConstants.IDLE_SHOOTER_RPM);
                }
            }
        }

        if(driveGamepad.dpad_up) {
            currentRPM = robotConstants.HIGH_GOAL_SHOOTER_RPM;
        } else if(driveGamepad.dpad_down) {
            currentRPM = robotConstants.POWER_SHOT_SHOOTER_RPM;
        }
    }

    public void setFlywheelPower (double power) {
        this.flywheel1.setPower(power);
        this.flywheel2.setPower(power);
    }

    public void setFlywheelVelocity(double desiredRPM) {
        this.flywheel1.setVelocity(rPMToTPS(desiredRPM));
    }

    public void setFlywheelVelocity(double desiredRPM1, double desiredRPM2) {
        this.flywheel1.setVelocity(rPMToTPS(desiredRPM1));
        this.flywheel2.setVelocity(rPMToTPS(desiredRPM2));
    }

    public double rPMToTPS(double rPM) {
        return (rPM * robotConstants.FLYWHEEL_TICKS_PER_ROTATION / 60);
    }

    public double returnRPM1() {
        lastTime = currentTime;
        currentTime = time.time();
        lastTick1 = currentTick1;
        currentTick1 = (flywheel1.getCurrentPosition() + flywheel2.getCurrentPosition()) / 2;

        RPM = (((Math.abs(this.currentTick1 - this.lastTick1))/robotConstants.FLYWHEEL_TICKS_PER_ROTATION) / ((Math.abs(this.currentTime - this.lastTime)) / 60));

        return (RPM);
    }

    public double returnRPM2() {
        lastTime = currentTime;
        currentTime = time.time();
        lastTick2 = currentTick2;
        currentTick2 = (flywheel1.getCurrentPosition() + flywheel2.getCurrentPosition()) / 2;

        RPM = (((Math.abs(this.currentTick2 - this.lastTick2))/robotConstants.FLYWHEEL_TICKS_PER_ROTATION) / ((Math.abs(this.currentTime - this.lastTime)) / 60));

        return (RPM);
    }
}

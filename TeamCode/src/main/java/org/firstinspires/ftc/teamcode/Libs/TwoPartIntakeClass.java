package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

public class TwoPartIntakeClass {
    private DcMotor mainIntake;
    private CRServo hopperIntake;

    MorganConstants robotConstants = new MorganConstants(null);

    public double driveSpeedLimiter = 1;

    public TwoPartIntakeClass(DcMotor mI, CRServo hI) {
        this.mainIntake = mI;
        this.hopperIntake = hI;
    }   //end of constructor

    /**
     * Method: intakeDriveControl(...)
     *  -   controls the intake with the controller
     * @param driveGamepad - the gamepad you want to use to drive the intake
     *      ___              ___
     *      ___ ____________ ___
     *  /    ^       (=)      y   \
     *  \ <    >   -    -  x    b  \
     *   \  v   _O_______O_  a    /
     *    \    /           \    /
     *     \_/              \_/
     */
    public void intakeDriveControl(Gamepad driveGamepad) {
        if(driveGamepad.right_trigger != 0) {
            mainIntake.setPower(robotConstants.MAIN_INTAKE_POWER*driveSpeedLimiter);
            hopperIntake.setPower(robotConstants.HOPPER_INTAKE_POWER*driveSpeedLimiter);
        } else if(driveGamepad.left_trigger != 0) {
            mainIntake.setPower(-(robotConstants.MAIN_INTAKE_POWER*driveSpeedLimiter));
            hopperIntake.setPower(-(robotConstants.HOPPER_INTAKE_POWER*driveSpeedLimiter));
        } else {
            mainIntake.setPower(0);
            hopperIntake.setPower(0);
        }

        if(driveGamepad.dpad_up) {
            driveSpeedLimiter = 1;
        } else if(driveGamepad.dpad_down) {
            driveSpeedLimiter = 0.5;
        }
    }   //end of intakeDriveControl(...)

    /**
     * Method: intakeEncoder(...)
     *  -   moves the intake wheels to intake with the encoder
     * @param mainPower - the power to the main intake wheels from 0 to 100
     * @param secondaryPower - the power to the secondary intake wheels from 0 to 100
     * @param ticks - the amount the intake should move
     */
    public void intakeEncoder(double mainPower, double secondaryPower, int ticks) {
        double correctedMainPower = Range.clip(mainPower, 0, 100)/100;
        double correctedSecondaryPower = Range.clip(secondaryPower, 0, 100)/100;

        mainIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mainIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mainIntake.setTargetPosition(ticks);

        mainIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mainIntake.setPower(correctedMainPower);
        hopperIntake.setPower(correctedSecondaryPower);

        while(mainIntake.isBusy());

        mainIntake.setPower(0);
        hopperIntake.setPower(0);
    }   //end of intakeEncoder(...)

    /**
     * Method: outtakeEncoder(...)
     *  -   moves the intake wheels to outtake with the encoder
     * @param mainPower - power to the main intake wheels from 0 to 100
     * @param secondaryPower - power to the secondary intake wheels from 0 to 100
     * @param ticks - the amount the intake should move
     */
    public void outtakeEncoder(double mainPower, double secondaryPower, int ticks) {
        double correctedMainPower = Range.clip(mainPower, 0, 100)/100;
        double correctedSecondaryPower = Range.clip(secondaryPower, 0, 100)/100;

        mainIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mainIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mainIntake.setTargetPosition(-ticks);

        mainIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mainIntake.setPower(correctedMainPower);
        hopperIntake.setPower(correctedSecondaryPower);

        while(mainIntake.isBusy());

        mainIntake.setPower(0);
        hopperIntake.setPower(0);
    }   //end of outtakeEncoder(...)
}   //end of TwoPartIntakeClass
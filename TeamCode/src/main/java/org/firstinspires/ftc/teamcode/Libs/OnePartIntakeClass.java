package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

public class OnePartIntakeClass {
    private DcMotor intakeMotor = null;

    MorganConstants robotConstants = new MorganConstants(null);

    public double driveSpeedLimiter = 1;

    public OnePartIntakeClass(DcMotor iM) {
        this.intakeMotor = iM;
    }   //end of construction

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
            intakeMotor.setPower(robotConstants.MAIN_INTAKE_POWER*driveSpeedLimiter);
        } else if(driveGamepad.left_trigger != 0) {
            intakeMotor.setPower(-(robotConstants.MAIN_INTAKE_POWER*driveSpeedLimiter));
        } else {
            intakeMotor.setPower(0);
        }

        if(driveGamepad.dpad_up) {
            driveSpeedLimiter = 1;
        } else if(driveGamepad.dpad_down) {
            driveSpeedLimiter = 0.5;
        }
    }   //end of intakeDriveControl(...)

    /**
     * Method: intakeEncoder(...)
     * @param power - the power that is applied to the intake from 0 to 100
     * @param ticks - the amount the intake should intake
     */
    public void intakeEncoder(double power, int ticks) {
        double correctedIntakePower = Range.clip(power, 0, 100)/100;

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setTargetPosition(ticks);

        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor.setPower(correctedIntakePower);

        while(intakeMotor.isBusy());

        intakeMotor.setPower(0);
    }   //end of intakeEncoder(...)

    /**
     * Method: outtakeEncoder(...)
     * @param power - the power that is applied to the intake from 0 to 100
     * @param ticks - the amount the intake should outtake
     */
    public void outtakeEncoder(double power, int ticks) {
        double correctedIntakePower = Range.clip(power, 0, 100)/100;

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setTargetPosition(-ticks);

        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor.setPower(correctedIntakePower);

        while(intakeMotor.isBusy());

        intakeMotor.setPower(0);
    }   //end of outtakeEncoder(...)
}

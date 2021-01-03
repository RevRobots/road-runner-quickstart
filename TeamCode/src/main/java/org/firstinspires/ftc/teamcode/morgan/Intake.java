package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {

    DcMotor intake;

    CRServo intakeWheels;

    int intakeControl = 1;

    public Intake (DcMotor i, CRServo iW) {

        intake = i;
        i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeWheels = iW;
    }

    public void intakeControl (Gamepad gamepad1) {

        if (gamepad1.dpad_up) {
            intakeControl = 1;
        } else if (gamepad1.dpad_left) {
            intakeControl = 2;
        } else if (gamepad1.dpad_right) {
            intakeControl = 3;
        }

        if (intakeControl == 1) {
            if (gamepad1.right_trigger != 0) {
                intake.setPower(gamepad1.right_trigger);
                intakeWheels.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                intake.setPower(-gamepad1.left_trigger);
                intakeWheels.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
                intakeWheels.setPower(0);
            }
        } else if (intakeControl == 2) {
            if (gamepad1.right_trigger != 0) {
                intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
                intakeWheels.setPower(0);
            }
        } else if (intakeControl == 3) {
            if (gamepad1.right_trigger != 0) {
                intakeWheels.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                intakeWheels.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
                intakeWheels.setPower(0);
            }
        }

    }

}

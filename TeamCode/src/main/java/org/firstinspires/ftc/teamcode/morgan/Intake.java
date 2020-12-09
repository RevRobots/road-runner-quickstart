package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {

    Gamepad gamepad1;

    DcMotor intake;

    CRServo intakeWheels;

    int intakeControl = 1;

    public Intake (Gamepad g1, DcMotor i, CRServo iW) {
        gamepad1 = g1;

        intake = i;
        i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeWheels = iW;
    }

    public void intakeControl () {

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
            }
        } else if (intakeControl == 2) {
            if (gamepad1.right_trigger != 0) {
                intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                intake.setPower(-gamepad1.left_trigger);
            }
        } else if (intakeControl == 3) {
            if (gamepad1.right_trigger != 0) {
                intakeWheels.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                intakeWheels.setPower(-gamepad1.left_trigger);
            }
        }

    }

}

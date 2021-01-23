package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {

    public DcMotor intake;

    int intakeControl = 1;

    public Intake (DcMotor i) {

        intake = i;
        i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            } else if (gamepad1.left_trigger != 0) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
            }
        } else if (intakeControl == 2) {
            if (gamepad1.right_trigger != 0) {
                intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
            }
        }

    }

}

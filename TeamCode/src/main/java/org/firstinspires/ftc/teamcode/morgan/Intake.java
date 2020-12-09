package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {

    Gamepad gamepad1;

    DcMotor intake;

    public Intake (Gamepad g1, DcMotor i) {
        gamepad1 = g1;

        intake = i;
        i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeControl () {

        if (gamepad1.right_trigger != 0) {
            intake.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger != 0) {
            intake.setPower(-gamepad1.left_trigger);
        }

    }

}

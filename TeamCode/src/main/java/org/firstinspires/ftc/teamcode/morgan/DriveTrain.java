package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveTrain {

    Gamepad gamepad1;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    double driveSpeedLimiter;

    public DriveTrain (Gamepad gamepad1, DcMotor lF, DcMotor rF, DcMotor lB, DcMotor rB) {
        leftFront = lF;
        rightFront = rF;
        leftBack = lB;
        rightBack = rB;
    }

    public void DriveTrainControl () {
        leftFront.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) / driveSpeedLimiter);
        rightFront.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x -gamepad1.left_stick_x) / driveSpeedLimiter);
        leftBack.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) / driveSpeedLimiter);
        rightBack.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) / driveSpeedLimiter);


    }

}

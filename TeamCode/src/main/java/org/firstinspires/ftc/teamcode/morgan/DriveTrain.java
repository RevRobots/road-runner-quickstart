package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveTrain {

    Gamepad gamepad1;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    double driveSpeedLimiter = 2;
    int speedCounter = 2;

    boolean brakeToggle = true;

    Toggles toggles = new Toggles();

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

        speedCounter = toggles.multiNumericToggle(1, 4, 'A', gamepad1);

        if (speedCounter == 1) {
            driveSpeedLimiter = 4;
        } else if (speedCounter == 2) {
            driveSpeedLimiter = 2;
        } else if (speedCounter == 3) {
            driveSpeedLimiter = 4/3;
        } else if (speedCounter == 4) {
            driveSpeedLimiter = 1;
        }

        brakeToggle = toggles.onOffToggle('B', gamepad1);

        if (brakeToggle == true) {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (brakeToggle == false) {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

}

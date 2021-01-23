package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class DriveTrain {

    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor leftOdometer, rightOdometer, frontOdometer;

    double driveSpeedLimiter = 2;
    int speedCounter = 2;

    boolean brakeToggle = true;

    Toggles toggles = new Toggles();

    public DriveTrain (DcMotor lF, DcMotor rF, DcMotor lB, DcMotor rB, DcMotor lO, DcMotor rO, DcMotor fO) {

        leftFront = lF;
        rightFront = rF;
        leftBack = lB;
        rightBack = rB;

        leftOdometer = lO;
        rightOdometer = rO;
        frontOdometer = fO;

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOdometer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveTrainControl (Gamepad gamepad1) {
        leftFront.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x - (gamepad1.left_stick_x)) / driveSpeedLimiter);
        rightFront.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x + (gamepad1.left_stick_x)) / driveSpeedLimiter);
        leftBack.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x + (gamepad1.left_stick_x)) / driveSpeedLimiter);
        rightBack.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x - (gamepad1.left_stick_x)) / driveSpeedLimiter);

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

    public void forwardsOdometry (double power, int tic) {
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOdometer.setTargetPosition(tic);
        rightOdometer.setTargetPosition(-tic);

        leftOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        while (leftOdometer.isBusy() && rightOdometer.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void backwardsOdometery (double power, int tic) {
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOdometer.setTargetPosition(-tic);
        rightOdometer.setTargetPosition(-tic);

        leftOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        while (leftOdometer.isBusy() && rightOdometer.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void rightOdometery (double power, int tic) {
        frontOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontOdometer.setTargetPosition(tic);

        frontOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(-power);

        while (frontOdometer.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void leftOdometery (double power, int tic) {
        frontOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontOdometer.setTargetPosition(tic);

        frontOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);

        while (frontOdometer.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void turnRightOdometery (double power, int tic) {
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOdometer.setTargetPosition(tic);
        rightOdometer.setTargetPosition(tic);

        leftOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);

        while (leftOdometer.isBusy() && rightOdometer.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void turnLeftOdometery (double power, int tic) {
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOdometer.setTargetPosition(tic);
        rightOdometer.setTargetPosition(tic);

        leftOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightOdometer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);

        while (leftOdometer.isBusy() && rightOdometer.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

}

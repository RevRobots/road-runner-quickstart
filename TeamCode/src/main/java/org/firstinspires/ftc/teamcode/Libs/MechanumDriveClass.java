package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dex.ProtoId;

public class MechanumDriveClass {

    /**
     * the basic mechanum wheel base layout
     */
    //      __________
    //     |\\      //|
    //     |          |
    //     |          |
    //      //______\\

    //the motors we want to use in the class
    private DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;
    private BNO055IMU imu = null;

    //the variables that help control the drive train
    public double driveSpeedLimiter = 0.5;
    public boolean reverseDriveControl = false;
    public boolean reverseFlag = false;

    ToggleClass reverseToggle = new ToggleClass();

    public MechanumDriveClass(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor, BNO055IMU inputImu) {
        this.leftFront = leftFrontMotor;
        this.rightFront = rightFrontMotor;
        this.leftBack = leftBackMotor;
        this.rightBack = rightBackMotor;

        this.imu = inputImu;
    }   //end of constructor

    /**
     * Method: mechanumDriveControl
     *  -   a way to control to drive train in tele-op
     * @param gamepad1 - the gamepad input that controls the wheels
     *      ___              ___
     *      ___ ____________ ___
     *  /    ^       (=)      y   \
     *  \ <    >   -    -  x    b  \
     *   \  v   _O_______O_  a    /
     *    \    /           \    /
     *     \_/              \_/
     *
     */
    public void mechanumDriveControl(Gamepad gamepad1) {
        if(reverseDriveControl == false) {
            leftFront.setPower(((-gamepad1.left_stick_y) + (gamepad1.right_stick_x) + (gamepad1.left_stick_x)) * driveSpeedLimiter);
            rightFront.setPower(((-gamepad1.left_stick_y) - (gamepad1.right_stick_x) - (gamepad1.left_stick_x)) * driveSpeedLimiter);
            leftBack.setPower(((-gamepad1.left_stick_y) + (gamepad1.right_stick_x) - (gamepad1.left_stick_x)) * driveSpeedLimiter);
            rightBack.setPower(((-gamepad1.left_stick_y) - (gamepad1.right_stick_x) + (gamepad1.left_stick_x)) * driveSpeedLimiter);
        } else if (reverseDriveControl == true) {
            leftFront.setPower(((gamepad1.left_stick_y) - (gamepad1.right_stick_x) - (gamepad1.left_stick_x)) * driveSpeedLimiter);
            rightFront.setPower(((gamepad1.left_stick_y) + (gamepad1.right_stick_x) + (gamepad1.left_stick_x)) * driveSpeedLimiter);
            leftBack.setPower(((gamepad1.left_stick_y) - (gamepad1.right_stick_x) + (gamepad1.left_stick_x)) * driveSpeedLimiter);
            rightBack.setPower(((gamepad1.left_stick_y) + (gamepad1.right_stick_x) - (gamepad1.left_stick_x)) * driveSpeedLimiter);
        }   //end of if(...) - else if(...)

        if(gamepad1.a) {
            driveSpeedLimiter = 1;
        } else if(gamepad1.b) {
            driveSpeedLimiter = 0.75;
        } else if(gamepad1.x) {
            driveSpeedLimiter = 0.5;
        } else if(gamepad1.y) {
            driveSpeedLimiter = 0.25;
        }   //end of if(...) - else if(...) - else if(...) - else if(...)

        reverseFlag = reverseToggle.buttonReleaseToggle(gamepad1.dpad_down, false);

        if(reverseFlag) {
            reverseDriveControl = true;
        } else if(!reverseFlag) {
            reverseDriveControl = false;
        }   //end of if(...) - else if(...)

        if(gamepad1.left_bumper) {
            driveLeftEncoder(37.5, 400);

        } else if(gamepad1.right_bumper) {
            driveRightEncoder(37.5, 400);
        }
    }   //end of mechanumDriveControl(...)

    /**
     * Method: driveForwardsEncoder(...)
     *  -   drives the robot forward using the drive encoders
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should move forward
     */
    public void driveForwardsEnocoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPower);
        rightFront.setPower(correctedPower);
        leftBack.setPower(correctedPower);
        rightBack.setPower(correctedPower);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }   //end of driveForwardsEnocoder()

    /**
     * Method: driveBackwardsEncoder(...)
     *  -   drives the robot backwards using the drive encoders
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should move backwards
     */
    public void driveBackwardsEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPower);
        rightFront.setPower(correctedPower);
        leftBack.setPower(correctedPower);
        rightBack.setPower(correctedPower);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }   //end of driveBackwardsEncoder(...)

    /**
     * Method: driveRightEncoder(...)
     *  -   drives the robot right using the drive encoders
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should move right
     */
    public void driveRightEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPower);
        rightFront.setPower(correctedPower);
        leftBack.setPower(correctedPower);
        rightBack.setPower(correctedPower);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }   //end of driveRightEncoder(...)

    /**
     * Method: driveRightEncoder(...)
     *  -   drives the robot left using the drive encoders
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should move left
     */
    public void driveLeftEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPower);
        rightFront.setPower(correctedPower);
        leftBack.setPower(correctedPower);
        rightBack.setPower(correctedPower);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }   //end of driveLeftEncoder(...)

    /**
     * Method: driveForwardsRightEncoder(...)
     *  -   drives the robot forward and right using the drive encoders
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should move forwards and right
     */
    public void driveForwardsRightEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPower);
        rightBack.setPower(correctedPower);

        while(leftFront.isBusy() && rightBack.isBusy());

        leftFront.setPower(0);
        rightBack.setPower(0);
    }   //end of driveForwardsRightEncoder(...)

    /**
     * Method: driveForwardsLeftEncoder(...)
     *  -   drives the robot forwards and left using the drive encoders
     *  THIS PROGRAM IS JUST A TEST AND MIGHT NOT BE CONSISTENT
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should move forwards and left
     */
    public void driveForwardsLeftEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setPower(correctedPower);
        leftBack.setPower(correctedPower);

        while(rightFront.isBusy() && leftBack.isBusy());

        rightFront.setPower(0);
        leftBack.setPower(0);
    }   //end of driveForwardsLeftEncoder(...)

    /**
     * Method: driveBackwardsRightEncoder(...)
     *  -   drives the robot backwards and right using the drive encoders
     *  THIS PROGRAM IS JUST A TEST AND MIGHT NOT BE CONSISTENT
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should move backwards and right
     */
    public void driveBackwardsRightEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setPower(correctedPower);
        leftBack.setPower(correctedPower);

        while(rightFront.isBusy() && leftBack.isBusy());

        rightFront.setPower(0);
        leftBack.setPower(0);
    }   //end of driveBackwardsRightEncoder(...)

    /**
     * Method: driveBackwardsLeftEncoder(...)
     *  -   drives the robot backwards and left using the drive encoders
     *  THIS PROGRAM IS JUST A TEST AND MIGHT NOT BE CONSISTENT
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should move backwards and left
     */
    public void driveBackwardsLeftEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPower);
        rightBack.setPower(correctedPower);

        while(leftFront.isBusy() && rightBack.isBusy());

        leftFront.setPower(0);
        rightBack.setPower(0);
    }   //end of driveBackwardsLeftEncoder(...)

    /**
     * Method: turnRightEncoder(...)
     *  -   turns the robot right using the drive encoders
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should turn right
     */
    public void turnRightEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPower);
        rightFront.setPower(correctedPower);
        leftBack.setPower(correctedPower);
        rightBack.setPower(correctedPower);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }   //end of turnRightEncoder(...)

    /**
     * Method: turnLeftEncoder(...)
     *  -   turns the robot left using the drive encoders
     * @param power - the power that the robot drives at from 0 to 100
     * @param ticks - how many ticks the robot should turn left
     */
    public void turnLeftEncoder(double power, int ticks) {
        double correctedPower = ((Range.clip(power, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(-ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPower);
        rightFront.setPower(correctedPower);
        leftBack.setPower(correctedPower);
        rightBack.setPower(correctedPower);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }   //end of turnLeftEncoder(...)

    /**
     * Method: autoForwardsTankDrive(...)
     *  - allows for different powers on the other side of the motor
     *  THIS PROGRAM IS JUST A TEST AND MIGHT NOT BE CONSISTENT
     * @param powerLeft - the amount of power the left side of the drive train gets
     * @param powerRight - the amount of power the right side of the drive train gets
     * @param ticks - how many ticks the robot should move
     */
    public void autoForwardsTankDrive(double powerLeft, double powerRight, int ticks) {
        double correctedPowerLeft = ((Range.clip(powerLeft, 0, 100))/100);
        double correctedPowerRight = ((Range.clip(powerRight, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(correctedPowerLeft);
        rightFront.setPower(correctedPowerRight);
        leftBack.setPower(correctedPowerLeft);
        rightBack.setPower(correctedPowerRight);

        while(leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }   //end of autoForwardsTankDrive

    /**
     * Method: autoBackwardsTankDrive(...)
     *  - allows for different powers on the other side of the motor
     *  THIS PROGRAM IS JUST A TEST AND MIGHT NOT BE CONSISTENT
     * @param powerLeft - the amount of power the left side of the drive train gets
     * @param powerRight - the amount of power the right side of the drive train gets
     * @param ticks - how many ticks the robot should move
     */
    public void autoBackwardsTankDrive(double powerLeft, double powerRight, int ticks) {
        double correctedPowerLeft = ((Range.clip(powerLeft, 0, 100))/100);
        double correctedPowerRight = ((Range.clip(powerRight, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-correctedPowerLeft);
        rightFront.setPower(-correctedPowerRight);
        leftBack.setPower(-correctedPowerLeft);
        rightBack.setPower(-correctedPowerRight);

        while(leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy());

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }   //end of autoBackwardsTankDrive

    public void driveForwardsPIDEncoder(double minimumPower, int ticks) {
        PIDLoopClass pid = new PIDLoopClass(0, 0, 0);
        double correctedPower = ((Range.clip(minimumPower, 0, 100))/100);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            leftFront.setPower(correctedPower + pid.pidControl(leftFront.getCurrentPosition(), ticks));
            rightFront.setPower(correctedPower + pid.pidControl(rightFront.getCurrentPosition(), ticks));
            leftBack.setPower(correctedPower + pid.pidControl(leftBack.getCurrentPosition(), ticks));
            rightBack.setPower(correctedPower + pid.pidControl(rightBack.getCurrentPosition(), ticks));
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }

}   //end of MechanumDriveClass
package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    DcMotor rotation;
    DcMotor flywheel;

    Servo ringPusher;

    double rotationLimiter;
    int rotationCounter;

    boolean fireMode = false;
    boolean isTriggerPressed;
    boolean launched = false;

    boolean isLoading = false;
    boolean isLoadingPressed;

    double zeroPowerPos = 0;

    Toggles toggles = new Toggles();
    ElapsedTime triggerTime = new ElapsedTime();
    ElapsedTime triggerDelayTime = new ElapsedTime();

    public Shooter (DcMotor r, DcMotor f, Servo rP) {

        rotation = r;
        flywheel = f;

        ringPusher = rP;
    }

    public void shooterControl (Gamepad gamepad2) {

        rotation.setPower((gamepad2.left_stick_y)/1.25);

        rotationCounter = toggles.multiNumericToggle(1, 4, 'y', gamepad2);

        if (rotationCounter == 1) {
            rotationLimiter = 4;
        } else if (rotationCounter == 2) {
            rotationLimiter = 2;
        } else if (rotationCounter == 3) {
            rotationLimiter = 4/3;
        } else if (rotationCounter == 4) {
            rotationLimiter = 1;
        }

        if (gamepad2.left_trigger != 0) {
            flywheel.setPower(0.85);
        } else {
            flywheel.setPower(0);
        }

        if (gamepad2.right_trigger != 0) {
            //trigger();
            ringPusher.setPosition(0.5);
        } else {
            ringPusher.setPosition(0);
        }

        isLoading = toggles.onOffToggle('D', gamepad2);

        if (isLoading == true) {
            zeroPowerPos = 0.5;
        } else if (isLoading == false) {
            zeroPowerPos = 0;
        }

        /*
        fireMode = toggles.onOffToggle('B', gamepad2);

        if (gamepad2.right_trigger != 0) {
            isTriggerPressed = false;
        }

        if (fireMode == false) {
            if (gamepad2.right_trigger != 0) {
                flywheel.setPower(1);
            } else {
                flywheel.setPower(0);
            }

            if (gamepad2.left_trigger != 0) {
                //trigger();
                ringPusher.setPosition(0.5);
            } else {
                ringPusher.setPosition(zeroPowerPos);
                triggerTime.reset();
            }
        } else if (fireMode == true) {

            if (gamepad2.right_trigger != 0) {

                if (isTriggerPressed == false) {
                    triggerDelayTime.reset();
                     isTriggerPressed = true;
                }

                flywheel.setPower(1);

                if (triggerDelayTime.milliseconds() >= 250) {

                    if (ringPusher.getPosition() == 0) {
                        //trigger();
                    }

                }

            }

        }

        isLoading = toggles.onOffToggle('D', gamepad2);

        if(isLoading == true) {
            zeroPowerPos = 0.5;
        } else if (isLoading == false) {
            zeroPowerPos = 0;
        }

         */

    }

    /*public void trigger () {

        triggerTime.reset();

        ringPusher.setPosition(0.5);

        if (triggerTime.milliseconds() >= 1000) {
            ringPusher.setPosition(0);
        }
    }*/

    public void rotateUp(double power, int tic) {
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotation.setTargetPosition(-tic);

        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotation.setPower(power);

        while(rotation.isBusy());

        rotation.setPower(0);
    }

    public void rotateDown(double power, int tic) {
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotation.setTargetPosition(tic);

        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotation.setPower(power);

        while(rotation.isBusy());

        rotation.setPower(0);
    }

    public void trigger () throws InterruptedException {
        ringPusher.setPosition(0.5);

        Thread.sleep(2000);

        ringPusher.setPosition(0);
    }

}
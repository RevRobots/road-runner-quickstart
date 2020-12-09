package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    Gamepad gamepad2;

    DcMotor rotation;
    DcMotor flywheel;

    Servo ringPusher;

    double rotationLimiter;
    int rotationCounter;

    boolean fireMode = false;
    boolean isTriggerPressed;

    Toggles toggles = new Toggles();
    ElapsedTime triggerTime = new ElapsedTime();
    ElapsedTime triggerDelayTime = new ElapsedTime();

    public Shooter (Gamepad g2, DcMotor r, DcMotor f, Servo rP) {
        gamepad2 = g2;

        rotation = r;
        flywheel = f;

        ringPusher = rP;
    }

    public void shooterControl () {

        if ((rotation.getCurrentPosition() >= 500 && gamepad2.left_stick_y < 0) || (rotation.getCurrentPosition() <=0 && gamepad2.left_stick_y > 0)) {
            rotation.setPower(0);
        } else {
            rotation.setPower((gamepad2.left_stick_y)/rotationLimiter);
        }

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

        fireMode = toggles.onOffToggle('B', gamepad2);

        if (gamepad2.right_trigger != 0) {
            isTriggerPressed = false;
        }

        if (fireMode == false) {
            if (gamepad2.right_trigger != 0) {
                flywheel.setPower(1);
            }

            if (gamepad2.left_trigger != 0 && ringPusher.getPosition() == 0) {
                trigger();
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
                        trigger();
                    }

                }

            }

        }

    }

    public void trigger () {

        triggerTime.reset();

        ringPusher.setPosition(0.5);

        if (triggerTime.milliseconds() >= 1000) {
            ringPusher.setPosition(0);
        }
    }

}
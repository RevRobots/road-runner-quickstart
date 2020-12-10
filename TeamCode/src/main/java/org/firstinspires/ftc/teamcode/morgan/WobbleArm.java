package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class WobbleArm {

    Gamepad gamepad2;

    DcMotor wobbleArm;

    CRServo finger;

    double wobbleArmLimiter = 2;
    boolean fullSpeed = false;

    Toggles toggles = new Toggles();

    public WobbleArm (Gamepad g2, DcMotor wA, CRServo f) {

        gamepad2 = g2;

        wobbleArm = wA;

        finger = f;

    }

    public void wobbleArmControl () {

        if (gamepad2.right_bumper) {
            wobbleArm.setPower(1/wobbleArmLimiter);
        } else if (gamepad2.left_bumper) {
            wobbleArm.setPower(-1/wobbleArmLimiter);
        }

        fullSpeed = toggles.onOffToggle('U', gamepad2);

        if (fullSpeed == true) {
            wobbleArmLimiter = 1;
        } else if (fullSpeed == false) {
            wobbleArmLimiter = 2;
        }

        if (gamepad2.x) {
            finger.setPower(1);
        } else if (gamepad2.a) {
            finger.setPower(-1);
        }

    }

}

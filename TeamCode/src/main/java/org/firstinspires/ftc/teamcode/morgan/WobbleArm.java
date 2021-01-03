package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class WobbleArm {

    DcMotor wobbleArm;

    CRServo finger;

    double wobbleArmLimiter = 2;
    boolean fullSpeed = false;

    Toggles toggles = new Toggles();

    public WobbleArm (DcMotor wA, CRServo f) {

        wobbleArm = wA;

        finger = f;

    }

    public void wobbleArmControl (Gamepad gamepad2) {

        if (gamepad2.right_bumper) {
            wobbleArm.setPower(1/wobbleArmLimiter);
        } else if (gamepad2.left_bumper) {
            wobbleArm.setPower(-1/wobbleArmLimiter);
        } else {
            wobbleArm.setPower(0);
        }

        fullSpeed = toggles.onOffToggle('U', gamepad2);

        if (fullSpeed == true) {
            wobbleArmLimiter = 4;
        } else if (fullSpeed == false) {
            wobbleArmLimiter = 2;
        }

        if (gamepad2.x) {
            finger.setPower(1);
        } else if (gamepad2.a) {
            finger.setPower(-1);
        } else {
            finger.setPower(0);
        }

    }

    public void armUp (double power, int tick) {

        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleArm.setTargetPosition(tick);

        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbleArm.setPower(power);

        while (wobbleArm.isBusy());

        wobbleArm.setPower(0);

    }

    public void armDown (double power, int tick) {

        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleArm.setTargetPosition(-tick);

        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbleArm.setPower(power);

        while (wobbleArm.isBusy());

        wobbleArm.setPower(0);

    }

    public void release (double power, int milliseconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();

        finger.setPower(-power);

        while (timer.milliseconds() >= milliseconds);

        finger.setPower(0);

    }

}

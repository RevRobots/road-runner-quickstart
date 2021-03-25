package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.ArmstrongConstants;
import org.firstinspires.ftc.teamcode.Libs.MechanumDriveClass;

@TeleOp (name = "Armstrong Drive Control")
@Disabled
public class ArmstrongDrive extends LinearOpMode {
    ArmstrongConstants robot;

    DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;
    DcMotor turret = null;
    DcMotor lift = null;
    DcMotor arm = null;
    CRServo leftClaw = null;
    CRServo rightClaw = null;

    MechanumDriveClass drive;

    @Override
    public void runOpMode() {
        robot = new ArmstrongConstants();

        leftFront = hardwareMap.get(DcMotor.class, robot.LEFT_FRONT);
        rightFront = hardwareMap.get(DcMotor.class, robot.RIGHT_FRONT);
        leftBack = hardwareMap.get(DcMotor.class, robot.LEFT_BACK);
        rightBack = hardwareMap.get(DcMotor.class, robot.RIGHT_BACK);

        turret = hardwareMap.get(DcMotor.class, robot.TURRET);

        lift = hardwareMap.get(DcMotor.class, robot.LIFT);

        arm = hardwareMap.get(DcMotor.class, robot.ARM);

        leftClaw = hardwareMap.get(CRServo.class, robot.LEFT_CLAW);
        leftClaw.setDirection(DcMotorSimple.Direction.REVERSE);
        rightClaw = hardwareMap.get(CRServo.class, robot.RIGHT_CLAW);

        drive = new MechanumDriveClass(leftFront, rightFront, leftBack, rightBack, null);

        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                drive.mechanumDriveControl(gamepad1);

                if(gamepad1.right_trigger != 0) {
                    arm.setPower(0.75);
                } else if(gamepad1.left_trigger != 0) {
                    arm.setPower(-0.25);
                } else {
                    arm.setPower(0);
                }

                if(gamepad1.right_bumper) {
                    leftClaw.setPower(1);
                    rightClaw.setPower(1);
                } else if(gamepad1.left_bumper) {
                    leftClaw.setPower(-1);
                    rightClaw.setPower(-1);
                }
            }
        }
    }
}

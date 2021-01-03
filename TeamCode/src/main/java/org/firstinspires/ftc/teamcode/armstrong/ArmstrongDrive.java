package org.firstinspires.ftc.teamcode.armstrong;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.morgan.DriveTrain;
import org.firstinspires.ftc.teamcode.morgan.RobotConfig;

@TeleOp (name = "Armstrong", group = "Tele-Op")
public class ArmstrongDrive extends OpMode {

    RobotConfig robotConfig;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DriveTrain driveTrain;

    @Override
    public void init () {

        robotConfig = new RobotConfig(hardwareMap);

        leftFront =  hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        driveTrain = new DriveTrain(leftFront, rightFront, leftBack, rightBack);

    }

    @Override
    public void start () {

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }

    @Override
    public void loop () {

        driveTrain.driveTrainControl(gamepad1);

    }

    @Override
    public void stop () {

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }

}

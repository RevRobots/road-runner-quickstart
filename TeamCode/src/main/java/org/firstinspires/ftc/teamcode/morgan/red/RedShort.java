package org.firstinspires.ftc.teamcode.morgan.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.morgan.DriveTrain;
import org.firstinspires.ftc.teamcode.morgan.Intake;
import org.firstinspires.ftc.teamcode.morgan.RobotConfig;
import org.firstinspires.ftc.teamcode.morgan.Shooter;
import org.firstinspires.ftc.teamcode.morgan.WobbleArm;

@Autonomous (name="Red Short", group = "red")
@Disabled
public class RedShort extends LinearOpMode {

    RobotConfig robotConfig;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    DcMotor intake;
    CRServo intakeWheel;

    DcMotor rotation, flywheel;
    Servo ringPusher;

    DcMotor wobbleGoalArm;
    CRServo finger;

    DriveTrain driveTrain;
    Intake intakeClass;
    Shooter shooter;
    WobbleArm wobbleArm;

    @Override
    public void runOpMode() {

        robotConfig = new RobotConfig(hardwareMap);

        robotConfig.initialize(leftFront, rightFront, leftBack, rightBack,
                intake, intakeWheel,
                rotation, flywheel, ringPusher,
                wobbleGoalArm, finger);

        driveTrain = new DriveTrain(leftFront, rightFront, leftBack, rightBack);
        intakeClass = new Intake(intake, intakeWheel);
        shooter = new Shooter(rotation, flywheel, ringPusher);
        wobbleArm = new WobbleArm(wobbleGoalArm, finger);

        waitForStart();



    }

}

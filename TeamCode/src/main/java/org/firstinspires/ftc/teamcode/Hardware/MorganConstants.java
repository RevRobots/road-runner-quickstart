package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MorganConstants {

    HardwareMap hardwareMap;

    //configuration names for the robot
    public static final String LEFT_FRONT = "leftFront";
    public static final String RIGHT_FRONT = "rightFront";
    public static final String LEFT_BACK = "leftBack";
    public static final String RIGHT_BACK = "rightBack";

    public static final String MAIN_INTAKE = "intake";
    public static final String HOPPER_INTAKE = "hopperIntake";

    public static final String FRONT_FLYWHEEL = "frontFlywheel";
    public static final String BACK_FLYWHEEL = "backFlywheel";
    public static final String RING_PUSHER = "ringPusher";

    public static final String WOBBLE_GOAL_ARM = "wobbleArm";
    public static final String WOBBLE_GOAL_FINGER = "finger";

    public static final String ARM_EXTENDED_LIMIT_SWITCH = "extendedLimit";
    public static final String ARM_RETRACTED_LIMIT_SWITCH = "retractedLimit";

    public static final String IMU = "imu";

    //Constant Variables For The Robot
    public static final double DRIVE_WHEELS_TICKS_PER_ROTATION = 1;
    public static final double DRIVE_WHEEL_RADIUS = 2;
    public static final double DRIVE_WHEEL_DIAMETER = DRIVE_WHEEL_RADIUS * 2;
    public static final boolean LEFT_FRONT_REVERSED = true;
    public static final boolean RIGHT_FRONT_REVERSED = false;
    public static final boolean LEFT_BACK_REVERSED = true;
    public static final boolean RIGHT_BACK_REVERSED = false;
    public static final boolean DRIVE_TRAIN_BRAKES = true;

    public static final double INTAKE_TICKS_PER_ROTATION = 537.6;
    public static final double MAIN_INTAKE_POWER = 0.75;
    public static final double HOPPER_INTAKE_POWER = 0.5;
    public static final boolean MAIN_INTAKE_REVERSED = true;
    public static final boolean HOPPER_INTAKE_REVERSED = false;
    public static final boolean INTAKE_BRAKES = false;

    public static final boolean RUN_DUAL_SHOOTER = false;
    public static final double FLYWHEEL_TICKS_PER_ROTATION = 28;
    public static final int HIGH_GOAL_SHOOTER_RPM = 10000;
    public static final int POWER_SHOT_SHOOTER_RPM = 6000;
    public static final int IDLE_SHOOTER_RPM = 3000;
    public static final boolean FLYWHEELS_REVERSED = false;
    public static final boolean FLYWHEELS_BRAKE = false;
    public static final double RING_PUSHER_RETRACTED = 0.25;
    public static final double RING_PUSHER_TRIGGERED = 0.75;

    public static final double WOBBLE_GOAL_ARM_TICKS_PER_ROTATION = 1;
    public static final int WOBBLE_GOAL_ARM_EXTENDED_TICK = 1000;
    public static final boolean WOBBLE_GOAL_ARM_REVERSED = false;
    public static final boolean WOBBLE_GOAL_FINGER_REVERSED = false;
    public static final boolean WOBBLE_GOAL_ARM_BRAKE = true;

    public static final String VUFORIA_LICENSE_KEY = "AQUWr4X/////AAABme38EPssRkvls9+q/BGPYgxKXBXELWHMdkTcCqUqHeyDpyXGWFLCTABgDXEMGe1EmsnDQxmJ7WQ069J3YSv+kOcfq3g2EnwZr2O3DujsIU1nT0aXgLlAtQU2r7wWAgHvR9ADO5pe/q7MzCyhjSTQLCgizGFLgmqfre0A9rjYcXYbYw11R3P7VRHnL3QHn3QH2oFVQfMb+dIzmZkfv0cd5qWvdhjovYF8hpZ/HT7veIa8ZQ9CIQ0541pxplXVud80z1xWpjFGJPaoQGO+xKWZ8E+Zlu7z5umiaV1+ChGeJ9pPyIJn0LsnoIHumZoYb4di4tFygMPVmH8ChsTlGJjaPBSCRBFjxzBqsXmBZY7eCa6S";

    /**
     * Constructor: RobotConstants()
     */
    public MorganConstants(HardwareMap mainHardwareMap) {
        this.hardwareMap = mainHardwareMap;
    }   //end of constructor

    public void initializeRobot(
            DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor,
            DcMotor mainIntakeMotor, CRServo hopperIntakeServo,
            DcMotorEx frontFlywheelMotor, DcMotorEx backFlywheelMotor, Servo ringPusherServo,
            DcMotor wobbleGoalArmMotor, CRServo fingerServo) {

        leftFrontMotor = hardwareMap.get(DcMotor.class, LEFT_FRONT);
        rightFrontMotor = hardwareMap.get(DcMotor.class, RIGHT_FRONT);
        leftBackMotor = hardwareMap.get(DcMotor.class, LEFT_BACK);
        rightBackMotor = hardwareMap.get(DcMotor.class, RIGHT_BACK);

        if(LEFT_FRONT_REVERSED) {
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(RIGHT_FRONT_REVERSED) {
            rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(LEFT_BACK_REVERSED) {
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(RIGHT_BACK_REVERSED) {
            rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if(DRIVE_TRAIN_BRAKES) {
            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        mainIntakeMotor = hardwareMap.get(DcMotor.class, MAIN_INTAKE);
        hopperIntakeServo = hardwareMap.get(CRServo.class, HOPPER_INTAKE);

        if(MAIN_INTAKE_REVERSED) {
            mainIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(HOPPER_INTAKE_REVERSED) {
            hopperIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if(INTAKE_BRAKES) {
            mainIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        frontFlywheelMotor = hardwareMap.get(DcMotorEx.class, FRONT_FLYWHEEL);
        backFlywheelMotor = hardwareMap.get(DcMotorEx.class, BACK_FLYWHEEL);
        ringPusherServo = hardwareMap.get(Servo.class, RING_PUSHER);

        if(FLYWHEELS_REVERSED) {
            frontFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if(FLYWHEELS_BRAKE) {
            frontFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        wobbleGoalArmMotor = hardwareMap.get(DcMotor.class, WOBBLE_GOAL_ARM);
        fingerServo = hardwareMap.get(CRServo.class, WOBBLE_GOAL_FINGER);

        if(WOBBLE_GOAL_ARM_REVERSED) {
            wobbleGoalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(WOBBLE_GOAL_FINGER_REVERSED) {
            wobbleGoalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if(WOBBLE_GOAL_ARM_BRAKE) {
            wobbleGoalArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}

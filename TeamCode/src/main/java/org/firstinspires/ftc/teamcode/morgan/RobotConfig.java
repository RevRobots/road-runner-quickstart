package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotConfig {

    HardwareMap hardwareMap;

    public static String leftFront = "leftFront";
    public static String rightFront = "rightFront";
    public static String leftBack = "leftBack";
    public static String rightBack = "rightBack";

    public static String leftOdometer = "leftBack";
    public static String rightOdometer = "rightBack";
    public static String frontOdometer = "leftFront";

    public static String intake = "intake";
    public static String intakeWheels = "intakeWheels";

    public static String rotation = "rotation";
    public static String flywheel = "flywheel";
    public static String ringPusher = "ringPusher";

    public static String wobbleArm = "wobbleArm";
    public static String finger = "finger";

    public static String distanceSensor = "distanceSensor";

    public static String imu = "imu";

    public static String revBlinkin = "led";

    public static double odometerTrackWidth = 14;
    public static double frontOdometerOffset;

    public RobotConfig (HardwareMap hwMap) {
        hardwareMap = hwMap;
    }

    public void initialize (
            DcMotor lF, DcMotor rF, DcMotor lB, DcMotor rB,
            DcMotor i, CRServo iW,
            DcMotor r, DcMotor f, Servo rP,
            DcMotor wGA, CRServo fn
    ) {

        lF = hardwareMap.dcMotor.get(leftFront);
        rF = hardwareMap.dcMotor.get(rightFront);
        lB = hardwareMap.dcMotor.get(leftBack);
        rB = hardwareMap.dcMotor.get(rightBack);

        i = hardwareMap.dcMotor.get(intake);
        iW = hardwareMap.crservo.get(intakeWheels);

        r = hardwareMap.dcMotor.get(rotation);
        f = hardwareMap.dcMotor.get(flywheel);
        rP = hardwareMap.servo.get(ringPusher);

        wGA = hardwareMap.dcMotor.get(wobbleArm);
        fn = hardwareMap.crservo.get(finger);

    }

}

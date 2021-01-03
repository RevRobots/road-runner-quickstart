package org.firstinspires.ftc.teamcode.morgan;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotConfig {

    HardwareMap hardwareMap;

    String leftFront = "leftFront";
    String rightFront = "rightFront";
    String leftBack = "leftBack";
    String rightBack = "rightBack";

    String leftOdometer = "leftBack";
    String rightOdometer = "rightBack";
    String frontOdometer = "leftFront";

    String intake = "intake";
    String intakeWheels = "intakeWheels";

    String rotation = "rotation";
    String flywheel = "flywheel";
    String ringPusher = "ringPusher";

    String wobbleArm = "wobbleArm";
    String finger = "finger";

    String imu = "imu";

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

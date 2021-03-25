package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

public class ArmClass {
    //actuators needed for the methods
    private DcMotor wobbleArm;
    private CRServo finger;
    private DigitalChannel retractedLimit;
    private DigitalChannel extendedLimit;

    //classes needed for the methods
    MorganConstants robotConstants = new MorganConstants(null);

    //variables needed for the methods
    public double armSpeedLimiter = 0.5;
    double zeroPower = 0;
    public double currentArmTicks;
    private boolean armPresetRunning = false;

    /**
     * Constructor: ArmClass(...)
     * @param wA - motor for the wobble goal arm
     * @param f - continuous rotation servo for the wobble goal finger
     */
    public ArmClass(DcMotor wA, CRServo f, DigitalChannel rL, DigitalChannel eL) {
        //sets the actuators
        this.wobbleArm = wA;
        this.finger = f;
        this.retractedLimit = rL;
        this.extendedLimit = eL;
    }   //end of constructor

    /**
     * Method: armDriveControl(...)
     *  -   controls the wobble arm with the controller
     * @param driveGamepad - the gamepad you want to use to drive the intake
     *      ___              ___
     *      ___ ____________ ___
     *  /    ^       (=)      y   \
     *  \ <    >   -    -  x    b  \
     *   \  v   _O_______O_  a    /
     *    \    /           \    /
     *     \_/              \_/
     */
    public void armDriveControl(Gamepad driveGamepad, double unaccountedArmTicks, boolean presets, boolean limitSwitches) {
        //drive control for the arm

        if(driveGamepad.right_bumper && extendedLimit.getState()) {
            wobbleArm.setPower(0.5);
        } else if(driveGamepad.left_bumper && retractedLimit.getState()) {
            wobbleArm.setPower(-0.5);
        } else {
            wobbleArm.setPower(0);
        }

        //arm speed controller
        if(driveGamepad.dpad_left) {
            armSpeedLimiter = 0.25;
        } else if (driveGamepad.dpad_right) {
            armSpeedLimiter = 0.5;
        }

        //controls the finger
        if(driveGamepad.x) {
            finger.setPower(-1);
        } else if(driveGamepad.a) {
            finger.setPower(1);
        } else {
            finger.setPower(0);
        }
    }   //end of armDriveControl(...)

    /**
     * Method: extendArmEncoder
     *  -   extends the wobble arm using encoders
     * @param power - the power the arm should move at from 0 to 100
     * @param ticks - the distance the arm should extend
     */
    public void extendArmEncoder(double power, int ticks) {
        //adjusts the power to the range needed by the motor
        double correctedArmPower = Range.clip(power, 0, 100)/100;

        //prepares the motor to move
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the motor to run
        wobbleArm.setTargetPosition(ticks);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(correctedArmPower);

        //waits for the motor to run
        while(wobbleArm.isBusy());

        //stops the motor
        wobbleArm.setPower(0);

        //keeps track of the arm encoder
        currentArmTicks = currentArmTicks + wobbleArm.getCurrentPosition();
    }   //end of extendArmEncoder(...)

    /**
     * Method: retractArmEncoder(...)
     *  -   retracts the wobble arm using encoders
     * @param power - the power the arm should move at from 0 to 100
     * @param ticks - the distance the arm should extend
     */
    public void retractArmEncoder(double power, int ticks) {
        //adjusts the power to the range needed by the motor
        double correctedArmPower = Range.clip(power, 0, 100);

        //prepares the motor
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the motor to run
        wobbleArm.setTargetPosition(-ticks);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(correctedArmPower);

        //waits for the motor to get to the set position
        while(wobbleArm.isBusy());

        //stops the motor
        wobbleArm.setPower(0);

        //adds the movement of the motor to the encoder tracker
        currentArmTicks = currentArmTicks + wobbleArm.getCurrentPosition();
    }   //end of retractArmEncoder(...)

    /**
     * Method: setArmPosition(...)
     *  -   extends the wobble arm using encoders
     * @param power - the power the arm should move at from 0 to 100
     * @param ticks - the distance the arm should extend
     * @param extraTicks - the unaccounted tick offset
     */
    public void setArmPosition(double power, int ticks, int extraTicks) {
        //adjusts the power to the range needed by the motor
        double correctedArmPower = Range.clip(power, 0, 100)/100;

        //prepares the motor
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the motor to run
        wobbleArm.setTargetPosition(ticks - extraTicks);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(correctedArmPower);
        armPresetRunning = true;

        /*//waits for the motor to get to the set position
        while(wobbleArm.isBusy());

        //stops the motor
        wobbleArm.setPower(0);

        //adds the movement of the motor to the encoder tracker
        currentArmTicks = currentArmTicks + wobbleArm.getCurrentPosition();

        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }   //end of setArmPosition(...)
}

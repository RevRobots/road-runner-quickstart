package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ActuatorTestClass {

    //External classes needed fot the methods
    private ElapsedTime movementTimer = new ElapsedTime();

    /**
     * Constructor: ActuatorTestClass()
     */
    public ActuatorTestClass() {

    }   //end of constructor

    /**
     * Method: motorEncoderTest(...)
     *  -   tests the motor movement with encoder measurements
     * @param testMotor - motor that needs to be tested
     * @param power - power level that is fed to the motor from 0 to 100
     * @param ticks - how many ticks the motor should move (Recommended is 2000)`
     */
    public void motorEncoderTest(DcMotor testMotor, double power, int ticks) {
        //corrects the input power to adjust the motor
        double correctedMotorPower = Range.clip(power, 0, 100)/100;

        //prepares the motor to run
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the motor to run
        testMotor.setTargetPosition(ticks);
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        testMotor.setPower(power);

        //waits for the motor to run to the set position
        while(testMotor.isBusy());

        //stops the motor
        testMotor.setPower(0);
    }   //end of motorEncoderTest(...)

    /**
     * Method: servoTest(...)
     *  -   tests the movement of a servo
     * @param testServo - servo that needs to be tested
     * @param minimumPosition - the lesser position to be tested
     * @param maximumPosition - the bigger position to be tested
     * @param intermissionTimerMilliseconds - the time between movements
     */
    public void servoTest(Servo testServo, double minimumPosition, double maximumPosition, int intermissionTimerMilliseconds) {
        //gets the starting position of the servo to move it back
        double startingPosition = testServo.getPosition();

        //resets the timer and tells the servo to move to maximum position
        movementTimer.reset();
        testServo.setPosition(minimumPosition);

        //waits for the supplied time
        while(movementTimer.milliseconds() < intermissionTimerMilliseconds);

        //resets the timer and tells the servo to move to maximum position
        movementTimer.reset();
        testServo.setPosition(maximumPosition);

        //waits for the supplied time
        while(movementTimer.milliseconds() < intermissionTimerMilliseconds);

        //returns the servo to the starting position
        testServo.setPosition(startingPosition);
    }   //end of servoTest(...)

    /**
     * Method: crservoTest(...)
     * @param testCRServo - continuous rotation servo that needs tested
     * @param power - power level that is fed to the continuous rotation servo
     * @param durationMilliseconds - duration of movement
     * @param restricted - whether the continuous rotation is limited to -0.5 to 0.5
     */
    public void continuousRotationServoTest(CRServo testCRServo, double power, int durationMilliseconds, boolean restricted) {
        //resets the timer and sets the servo to a power
        movementTimer.reset();
        testCRServo.setPower(power);

        //waits for the supplied time
        while(movementTimer.milliseconds() < durationMilliseconds);

        //stops the servo
        testCRServo.setPower(0);
    }   //end of continuousRotationServoTest(...)

}

package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.util.Range;

public class PIDLoopClass {

    private double kP;
    private double kI;
    private double kD;

    public PIDLoopClass(double kp, double ki, double kd) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
    }   //end of constructor

    /**
     * Method: pidControl
     *  -   a pid control method for better outputs
     * @param input - the sensor value that measures how much something changed
     * @param goal - the goal for the change
     * @return - gives back a value to change the output by
     */
    public double pidControl(double input, double goal) {
        double error = goal - input;
        double integral = 0;
        double derivative = 0;

        double correction = ((kP*error) + (kI*integral) + (kD*derivative));

        return correction;
    }   //end of pidControl(...)
}

package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MorganConstants;

public class RingPusherClass {
    Servo ringPusher = null;

    MorganConstants robotConstants = new MorganConstants(null);

    public RingPusherClass(Servo rP) {
        this.ringPusher = rP;
    }   //end of constructor

    /**
     * Method: ringPusherControl(...)
     *  -   a way to control the ring pusher in tele-op
     * @param driveGamepad - the controller used to control the ring pusher
     */
    public void ringPusherControl(Gamepad driveGamepad) {
        if(driveGamepad.right_trigger != 0) {
            ringPusher.setPosition(robotConstants.RING_PUSHER_TRIGGERED);
        } else {
            ringPusher.setPosition(robotConstants.RING_PUSHER_RETRACTED);
        }
    }   //end of ringPusherControl(..)

    /**
     * Method: trigger(...)
     *  -   moves the ring pusher to fire and moves it back
     * @param milliseconds - the delay in milliseconds before the servo retracts
     */
    public void trigger(int milliseconds) {
        ElapsedTime timer = new ElapsedTime();

        ringPusher.setPosition(robotConstants.RING_PUSHER_TRIGGERED);

        while(timer.milliseconds() < milliseconds);

        ringPusher.setPosition(robotConstants.RING_PUSHER_RETRACTED);
    }
}

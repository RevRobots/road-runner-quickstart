package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.Gamepad;

public class MovementClass {
    public String text = "The Robot Is Norminal";

    public MovementClass() {

    }   //end of constructor

    public String movement(Gamepad gamepad) {
        if(gamepad.start) {
            text = "JUMP!";
        }

        return text;
    }
}

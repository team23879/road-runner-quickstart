package org.firstinspires.ftc.teamcode.robonauts;

import static com.sun.tools.doclint.HtmlTag.B;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmRotation {
    DcMotor motor = null;
    public ArmRotation (LinearOpMode opMode) {
        //motor = opMode.hardwareMap.dcMotor.get("TestMotor");
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setTargetPosition(0);
        //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motor.setPower(0.4);
    }

    public int setArmPower(boolean up) {
        int position =0;
        if (up) {
            position = motor.getCurrentPosition() + 50;
        } else {
            position = motor.getCurrentPosition() - 60;
        }
        motor.setTargetPosition(position);
        return position;
    }

}

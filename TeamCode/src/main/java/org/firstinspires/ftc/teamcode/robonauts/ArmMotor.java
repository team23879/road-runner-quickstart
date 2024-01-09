package org.firstinspires.ftc.teamcode.robonauts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class ArmMotor extends OpMode {
    public DcMotor TestMotor;
    public CRServo secondservo;


    @Override
    public void init() {
        secondservo = hardwareMap.get(CRServo.class, "second_servo");
        TestMotor = hardwareMap.get(DcMotor.class, "TestMotor");
    }

    @Override
    public void loop() {
        telemetry.addData("second_servo", gamepad1.right_stick_y);
        telemetry.addData("TestMotor", gamepad1.left_stick_y);

        TestMotor.setPower(gamepad1.left_stick_y);

        secondservo.setPower(gamepad1.right_stick_y);

    }
}


package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestM extends OpMode {
    public DcMotor TestMotor;


    @Override
    public void init() {
        TestMotor = hardwareMap.get(DcMotor.class, "TestMotor");
    }

    @Override
    public void loop() {
        telemetry.addData("TestMotor", gamepad1.left_stick_y);

        TestMotor.setPower(  gamepad1.left_stick_y);

        telemetry.update();

    }

}


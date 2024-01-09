package org.firstinspires.ftc.teamcode.robonauts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.TestMotor;

@TeleOp
public class adithyamotor extends LinearOpMode {
    public DcMotor TestMotor;
    DcMotorEx motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "TestMotor");

        // Reset the encoder during initialization
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        // Set the motor's target position to 300 ticks
        motor.setTargetPosition(300);

        // Switch to RUN_TO_POSITION mode
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (-motor.getCurrentPosition() < motor.getTargetPosition()) {

            // Start the motor moving by setting the max velocity to 200 ticks per second
            motor.setVelocity(200);
            telemetry.addData("velocity", motor.getVelocity());
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.addData("target position", motor.getTargetPosition());

            telemetry.addData("is at target", !motor.isBusy());
            telemetry.update();
        }
        motor.setPower(0);
        // While the Op Mode is running, show the motor's status via telemetry
        while (motor.isBusy()) {
            telemetry.addData("velocity", -motor.getVelocity());
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.addData("is at target", !motor.isBusy());
            telemetry.update();
        }
        }
    }


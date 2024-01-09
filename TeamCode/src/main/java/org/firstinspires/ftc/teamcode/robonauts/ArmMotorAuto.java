package org.firstinspires.ftc.teamcode.robonauts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.TestMotor;

@TeleOp
public class ArmMotorAuto extends LinearOpMode {
    public DcMotor TestMotor;
    public CRServo secondservo;

    private double initialPower=-0.33;
    private double aligntoBoardPower = 0.22;

    private double groundPower = 0.11;

    private boolean aligntoGround = false;
    private boolean aligntoBoard = false;

    private boolean initialPostition=true;

    private final double TICK_COUNT=1425;
    @Override
    public void runOpMode() throws InterruptedException {
        secondservo = hardwareMap.get(CRServo.class, "second_servo");
        TestMotor = hardwareMap.get(DcMotor.class, "TestMotor");
        TestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //ArmRotation armRotation = new ArmRotation(this);

        waitForStart();
        TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int initpos = TestMotor.getCurrentPosition();
        //while(opModeIsActive()) {
            //aligntoBoard = gamepad1.a;
            double oneThreeFiveAngel = TICK_COUNT / 4;
            telemetry.addData("before Position", initpos);

            //if (gamepad1.a) {
                TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                int newTarget = TestMotor.getTargetPosition() + (int) oneThreeFiveAngel;
                TestMotor.setTargetPosition(newTarget);
                TestMotor.setPower(1);
                TestMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //}
            while (opModeIsActive()) {
                telemetry.addData("before Position", initpos);

                telemetry.addData("Motor Power", TestMotor.getPower());
                telemetry.addData("Position", TestMotor.getCurrentPosition());

                telemetry.update();
            }
        //}

    }

}


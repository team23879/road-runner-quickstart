package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
@Config
public class PIDFArm extends OpMode {
    private final double zeroOffset = 30;

    private PIDController pidController=null;
    public static  double kp=0;//0.77;
    public static  double ki=0;//0.003;
    public static  double kd=0.001;//0.0001;

    public static double kf=0.01;//0.03;

    public static int targetMotorPosition=0;

    //public static int targetDeg=0;
    private final double ticksPerDegree=537.7 / 360;
    public  Encoder armMotorEncoder;
    DcMotorEx armMotor = null;
    @Override
    public void init() {
        pidController = new PIDController(kp, ki,kd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.get(DcMotorEx.class, "TestMotor");
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        pidController.setPID(kp,ki,kd);
        int armPos =  armMotor.getCurrentPosition();
        //double targetPositionTicks = ((targetDeg-zeroOffset)*ticksPerDegree)/armPos;
        double pid = pidController.calculate(armPos, targetMotorPosition);
       // double feedforward = Math.cos(Math.toRadians(targetMotorPosition / ticksPerDegree)) * kf;
        double angel = armPos / ticksPerDegree + zeroOffset;
        double feedforward = Math.sin(Math.toRadians(angel)) * kf;
        double armPositionInDeg = armPos/ticksPerDegree + zeroOffset;
        double power = pid + feedforward;
        armMotor.setPower(power);
        telemetry.addData("pid", pid);

        telemetry.addData("ff", feedforward);
        telemetry.addData("angel", angel);
        telemetry.addData("power", power);

        telemetry.addData("current pos", armPos);
        telemetry.addData("Arm position in degrees ", armPositionInDeg);
        //telemetry.addData("target", targetDeg);

        telemetry.update();
    }
}

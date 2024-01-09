package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDSlides extends LinearOpMode {
    private PIDController pidController=null;
    public static  double kp=0;//0.77;
    public static  double ki=0;//0.003;
    public static  double kd=0;//0.004;

    public static double kf=0;//0.03;

    public static int targetMotorPosition=0;

    //public static int targetDeg=0;
    private final double ticksPerDegree=1425.1 / 360;
    public Encoder armMotorEncoder;
    DcMotorEx leftMotor = null;
    DcMotorEx rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotorEx.class, "par0");
        rightMotor = hardwareMap.get(DcMotorEx.class, "par1");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

    }
}

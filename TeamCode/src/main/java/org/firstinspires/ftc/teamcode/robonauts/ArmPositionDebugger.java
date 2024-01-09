package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class ArmPositionDebugger extends LinearOpMode {
    private final double zeroOffset = 23.0;

    DcMotorEx armMotor = null;
    private final double ticksPerDegree=1425 / 360;

    public double lowPosition = -20;
    public static double highPosition=-300;
    public static  double kp=0.77;
    public static  double ki=0.003;
    public static  double kd=0.004;

    public static double kf=0.03;
    private PIDFController pidController=null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.get(DcMotorEx.class, "TestMotor");
        pidController = new PIDFController(kp, ki,kd, kf);
        telemetry.addData("Current Postion before init : ", armMotor.getCurrentPosition());

        waitForStart();
        while (opModeIsActive()) {
            int armPos =  armMotor.getCurrentPosition();
            telemetry.addData("current pos", armPos);
            if (gamepad1.b) {

                telemetry.addData("Current Postion Game Pad B: ", armMotor.getCurrentPosition());
                double power = getPower(highPosition);
                armMotor.setPower(-power);
                telemetry.addData("Power to go to -800 : ", power);
            }


            telemetry.update();
        }
    }

    public double getPower(double targetMotorPosition) {
        pidController.setPIDF(kp,ki,kd, kf);
        int armPos =  armMotor.getCurrentPosition();
        double pid = pidController.calculate(armPos, targetMotorPosition);
        double feedforward = Math.cos(Math.toRadians(targetMotorPosition / ticksPerDegree)) * kf;
        double power = pid * feedforward;
        return power;
    }

    public PIDFController getPidController() {
        return pidController;
    }
}

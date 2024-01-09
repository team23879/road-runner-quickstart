package org.firstinspires.ftc.teamcode.robonauts;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm
{
    private static final double ENCODER_CPR = 1425.1;
    private static final double GEAR_RATIO = 1.0;
    private static final double ARM_TICKS_PER_DEGREE = ENCODER_CPR * GEAR_RATIO / 360.0;
    private static final double MAX_ARM_HOLDING_POWER = 0.03;
    private static final double ZERO_OFFSET = 45.0;
    private static final double ARM_MIN_POS = 45.0;
    private static final double ARM_MAX_POS = 270.0;
    public static  double kp=0.77;
    public static  double ki=0.003;
    public static  double kd=0.004;
    private final DcMotorEx armMotor;
    private final PIDController controller;

    private double targetPosInDegrees;
    private double powerLimit;

    /**
     * Constructor: create and initialize everything required by the arm subsystem including the arm motor and
     * PID controller.
     */
    public Arm(DcMotorEx armMotor)
    {
        // Create arm motor here and initialize it.
        this.armMotor = armMotor;
        // Create PID controller for the arm and initialize it with proper PIDF coefficients.
        controller = new PIDController(0.77, 0.003, 0.004);
    }

    /**
     * This method must be call periodically so that it will do PID control of the arm towards target position and
     * hold it.
     */
    public void armTask()
    {
        double targetPosInTicks   = (targetPosInDegrees - ZERO_OFFSET) * ARM_TICKS_PER_DEGREE;
        double currPosInTicks = armMotor.getCurrentPosition();
        double pidOutput = controller.calculate(currPosInTicks, targetPosInTicks);
        // This ff is assuming arm at horizontal position is 90-degree.
        double ff = MAX_ARM_HOLDING_POWER * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
        double power = pidOutput + ff;
        // Clip power to the range of -powerLimit to powerLimit.
        power = power < -powerLimit ? -powerLimit : power > powerLimit ? powerLimit : power;
        armMotor.setPower(power);
    }

    /**
     * This method is typically called by autonomous to determine when the arm has reached target so that it can move
     * on to do the next thing.
     *
     * @param toleranceInDegrees specifies the tolerance in degrees to declare On Target.
     * @return true if arm is on target within tolerance, false otherwise.
     */
    public boolean isOnTarget(double toleranceInDegrees)
    {
        double currPosInDegrees = getPosition();
        return Math.abs(targetPosInDegrees - currPosInDegrees) <= toleranceInDegrees;
    }

    /**
     * This method can be called by autonomous or teleop to set the arm target. Typically, in TeleOp, one can react
     * to a button press and call this method to move the arm to a preset position.
     *
     * @param targetPosInDegrees specifies the target position in degrees.
     * @param powerLimit specifies the maximum power for the arm movement.
     */
    public void setPosition(double targetPosInDegrees, double powerLimit)
    {
        this.targetPosInDegrees = targetPosInDegrees;
        this.powerLimit = Math.abs(powerLimit);
    }

    /**
     * This method is typically used by TeleOp to control the arm movement by the value of the joystick so that the
     * speed of the arm movement can be controlled by the joystick. Since this is PID controlled, the arm will slow
     * down when approaching the lower and upper limits even though the joystick is pushed to max position.
     *
     * @param power specifies the maximum power for the arm movement.
     */
    public void setPower(double power)
    {
        if (power > 0.0)
        {
            // Move arm towards max position with specified power.
            setPosition(ARM_MAX_POS, power);
        }
        else if (power < 0.0)
        {
            // Move arm towards min position with specified power.
            setPosition(ARM_MIN_POS, power);
        }
        else
        {
            // Hold arm position without power limit.
            setPosition(getPosition(), 1.0);
        }
    }

    /**
     * This method translates encoder ticks to real world arm position in degrees.
     *
     * @param encoderTicks specifies the motor encoder ticks.
     * @return translated arm position in degrees.
     */


    /**
     * This method returns the arm position in real world degrees.
     *
     * @return arm position in degrees.
     */
    public double getPosition()
    {
        return ticksToRealWorldDegrees(armMotor.getCurrentPosition());
    }

    public double ticksToRealWorldDegrees(double ticks)
    {
        return ticks / ARM_TICKS_PER_DEGREE + ZERO_OFFSET;
    }
}





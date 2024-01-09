package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class LineToX extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Vector2d myVector = new Vector2d(0, 0);
        Pose2d pose2d = new Pose2d(myVector, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose2d);

        telemetry.addData("before x", drive.pose.position.x);
        telemetry.addData("before y", drive.pose.position.y);
        telemetry.addData("before heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        //Action action = mecanumDrive.actionBuilder(new Pose2d( new Vector2d(30,30), 90)).lineToX(30).build();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(-48)
                        //.splineTo(new Vector2d(2, 2), 0)
                        //.splineTo(new Vector2d(60, 0), Math.PI)
                        .build());
        TelemetryPacket telemetryPacket = new TelemetryPacket(true);
        telemetryPacket.field().strokeLine(0, 0, 30, 0);
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();        //action.run(telemetryPacket);
    }
}

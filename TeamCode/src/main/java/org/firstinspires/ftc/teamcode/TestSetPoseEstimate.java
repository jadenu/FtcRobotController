package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

@TeleOp(name="Test Set Pose Estimate")
public class TestSetPoseEstimate extends OpMode {

    Telemetry.Log log;

    org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive drive;

    @Override
    public void init() {
        log = telemetry.log();

        drive = new org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
    }

    @Override
    public void loop() {
        drive.setPoseEstimate(new Pose2d(30, 15, 10));

        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        drive.setPoseEstimate(new Pose2d(-30, 45, 90));

        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        drive.setPoseEstimate(new Pose2d(45, -39, 180));

        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

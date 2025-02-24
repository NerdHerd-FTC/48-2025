package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Right Side Park", preselectTeleOp = "FO Mecanum Drive with Slides")
public class rightPark extends LinearOpMode {

    private final double DELAY = 1;

    @Override
    public void runOpMode(){
        Pose2d initPose = new Pose2d(12,-64,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initPose);

        Action park = drive.actionBuilder(initPose)
                .setTangent(0)
                .lineToX(60)
                .build();

        telemetry.addLine("skibidi");
        telemetry.addData("DELAY", DELAY);
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;
        Actions.runBlocking(
            new SequentialAction(
                    new SleepAction(DELAY),
                    park
            )
        );
    }
}

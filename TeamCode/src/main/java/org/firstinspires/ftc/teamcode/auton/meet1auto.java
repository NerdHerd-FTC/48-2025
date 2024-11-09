package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ParallelAction;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Meet 1 Auto")
public class meet1auto extends swingArmActions {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initPose = new Pose2d(12,-60,Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initPose);

        Claw claw = new Claw(hardwareMap);
        Actuator actuator = new Actuator(hardwareMap);
        Swing swing = new Swing(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);

        TrajectoryActionBuilder path1 = drive.actionBuilder(initPose)
                .splineToConstantHeading(new Vector2d(0,-41.0),Math.toRadians(90));
        TrajectoryActionBuilder moveBack = path1.fresh()
                .splineToConstantHeading(new Vector2d(0,-41.5),Math.toRadians(90));
        TrajectoryActionBuilder path2 = path1.fresh()
                .splineToConstantHeading(new Vector2d(60,-60),Math.toRadians(90));

        Action actionPath1 = path1.build();
        Action actionMoveBack = moveBack.build();
        Action actionPath2 = path2.build();

        Actions.runBlocking(claw.closeClaw());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("yeag");
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        swing.moveSwingScore(),
                        new ParallelAction(
                                actionPath1,
                                actuator.moveActuatorScore()
                        ),
                        pivot.pivotDown(),
                        new SleepAction(1),
                        actionMoveBack,
                        new ParallelAction(
                                actionPath2,
                                actuator.moveActuatorDown(),
                                pivot.pivotUp()
                        ),
                        swing.moveSwingDown()
                )
        );
    }
}

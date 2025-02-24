package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Left Auto", preselectTeleOp = "FO Mecanum Drive with Slides")
@Config
public class iltAutoLeft extends linearSlidesActions {
    private final double startDelay = 0.0;
    // parking includes moving across the field
    // delay until alliance partner has parked
    private final double parkDelay = 1.0;

    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d initPose = new Pose2d(-12,-64,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initPose);

        IntakeCactus cactus = new IntakeCactus();
        IntakePivot intakePivot = new IntakePivot();
        OuttakeClaw claw = new OuttakeClaw();
        OuttakePivot outtakePivot = new OuttakePivot();
        IntakeSlide intakeSlide = new IntakeSlide();
        OuttakeSlide outtakeSlide = new OuttakeSlide();
        OuttakeSpin spin = new OuttakeSpin();

        VelConstraint constraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 7;
            }
        };

        // this pose is used to position in front of the basket
        // TODO: position this to score
        Pose2d scorePose = new Pose2d(-50, -50, Math.toRadians(45));
        Pose2d scorePoseTwo = new Pose2d(-45.5, -54.5, Math.toRadians(45));

        // this pose is used to position in front of the spike
        // TODO: position this to intake
        Pose2d spikePose = new Pose2d(-49.00, -44.00, Math.toRadians(-90.00));
        Pose2d spikeForwardPose = new Pose2d(spikePose.position.x, spikePose.position.y + 0.50, spikePose.heading.toDouble());
        double forwardMoveY = spikeForwardPose.position.y;

        Action startToBasket = drive.actionBuilder(initPose)
                .splineToLinearHeading(scorePose, Math.toRadians(-135.00))
                .build();

        Action basketToFirstSpike = drive.actionBuilder(scorePose)
                .splineToLinearHeading(spikePose, Math.toRadians(90.00))
                .build();

        Action forwardMove = drive.actionBuilder(spikePose)
                .setTangent(Math.toRadians(-90))
                .lineToY(forwardMoveY)
                .build();

        Action firstSpikeToBasket = drive.actionBuilder(spikePose)
                .splineToLinearHeading(scorePoseTwo, Math.toRadians(-135.00))
                .build();

        Action park = drive.actionBuilder(scorePoseTwo)
                .splineToLinearHeading(new Pose2d(44,-60,Math.toRadians(90)),Math.toRadians(0))
                .build();


        Actions.runBlocking(new SequentialAction(spin.horizontal(),claw.close(),outtakePivot.intake(),intakePivot.up(),intakeSlide.in()));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("yeag");
            telemetry.update();
        }
        waitForStart();

        if(isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(startDelay),
                        // go to basket
                        new ParallelAction(
                                startToBasket,
                                outtakeSlide.score()
                        ),
                        // score
                        outtakePivot.score(),
                        new SleepAction(1),
                        claw.open(),
                        new SleepAction(1),
                        outtakePivot.intake(),
                        new SleepAction(0.5),
                        // go to spike mark
                        new ParallelAction(
                                outtakeSlide.floor(),
                                basketToFirstSpike
                        ),
                        // pick up
                        outtakePivot.floor(),
                        new SleepAction(1),
                        forwardMove,
                        claw.close(),
                        new SleepAction(0.5),
                        outtakePivot.intake(),
                        new SleepAction(1),
                        new ParallelAction(
                                firstSpikeToBasket,
                                outtakeSlide.score()
                        ),
                        outtakePivot.score(),
                        new SleepAction(1),
                        claw.open(),
                        new SleepAction(1),
                        outtakePivot.intake(),
                        new SleepAction(0.5),
                        new ParallelAction(
                            outtakeSlide.floor(),
                            park
                        )
                )
        );
    }
}

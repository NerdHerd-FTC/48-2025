package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Left Auto")
public class iltAutoLeft extends linearSlidesActions {

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

        Action path1 = drive.actionBuilder(initPose)
                    .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)), Math.toRadians(-135.00))
                    .build();

        Actions.runBlocking(new SequentialAction(spin.horizontal(),claw.open(),outtakePivot.intake(),intakePivot.up(),intakeSlide.in()));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("yeag");
            telemetry.update();
        }
        waitForStart();

        if(isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        path1,
                        claw.close()
                )
        );
    }
}

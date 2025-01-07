package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="FO Mecanum Drive with Slides")
@Disabled
public class mecanumDriveFO extends LinearOpMode {

    @Override
    public void runOpMode() {
        // import drive and slides
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        linearSlides linearSlides = new linearSlides();

        // init motors and servos
        DcMotorEx outtakeSlideL = hardwareMap.get(DcMotorEx.class, "outtakeSlideL");
        DcMotorEx outtakeSlideR = hardwareMap.get(DcMotorEx.class, "outtakeSlideR");
        Servo outtakePivotL = hardwareMap.get(Servo.class, "outtakePivotL");
        Servo outtakePivotR = hardwareMap.get(Servo.class, "outtakePivotR");
        Servo outtakeClaw = hardwareMap.get(Servo.class,"outtakeClaw");
        Servo intakeSlideL = hardwareMap.get(Servo.class,"intakeSlideL");
        Servo intakeSlideR = hardwareMap.get(Servo.class,"intakeSlideR");
        Servo intakeTopPivotL = hardwareMap.get(Servo.class,"intakeTopPivotL");
        Servo intakeTopPivotR = hardwareMap.get(Servo.class,"intakeTopPivotR");
        Servo intakeBottomPivotL = hardwareMap.get(Servo.class,"intakeBottomPivotL");
        Servo intakeBottomPivotR = hardwareMap.get(Servo.class,"intakeBottomPivotR");
        CRServo intakeCactus = hardwareMap.get(CRServo.class,"intakeCactus");


        // set target position for RUN_TO_POSITION
        outtakeSlideL.setTargetPosition(outtakeSlideL.getCurrentPosition());
        outtakeSlideR.setTargetPosition(outtakeSlideR.getCurrentPosition());

        // set mode
        outtakeSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set velocity for RUN_TO_POSITION
        outtakeSlideL.setVelocity(org.firstinspires.ftc.teamcode.teleop.linearSlides.SLIDE_MAX_VEL);
        outtakeSlideR.setVelocity(org.firstinspires.ftc.teamcode.teleop.linearSlides.SLIDE_MAX_VEL);

        // set ZeroPowerBehavior
        outtakeSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set directions
        // TODO: set these correctly



        double robotOffset = 0;
        String robotDirection = "Forward";
        telemetry.addData("Robot Offset", robotOffset);
        telemetry.addData("Robot Direction", robotDirection);

        while (opModeInInit()) {

            telemetry.addLine("Press the face button in the direction of the robot");

            if (gamepad1.y){
                robotOffset = 0;
                robotDirection = "Forward";
            } else if (gamepad1.b){
                robotOffset = Math.toRadians(-90);
                robotDirection = "Right";
            } else if (gamepad1.a){
                robotOffset = Math.toRadians(180);
                robotDirection = "Reverse";
            } else if (gamepad1.x){
                robotOffset = Math.toRadians(90);
                robotDirection = "Left";
            }

            telemetry.addLine("Press BACK to reset ALL MECHANISM positions");

            if (gamepad1.back){
                // TODO: add position reset here
            }


            telemetry.addData("Robot Offset", robotOffset);
            telemetry.addData("Robot Direction", robotDirection);
            telemetry.addData("Left Outtake Slide Position", outtakeSlideL.getCurrentPosition());
            telemetry.addData("Right Outtake Slide Position", outtakeSlideR.getCurrentPosition());
            telemetry.addLine("ready to go");
            telemetry.update();
        }
        waitForStart();

        drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);

        while (opModeIsActive()){

            drive.updatePoseEstimate();

            double heading = drive.pose.heading.toDouble();

            //take a look at an explanation of this math at https://www.desmos.com/calculator/grw2phgz3j
            //these values are flipped due to the rotation of roadrunner's coordinate system
            Vector2d stickPos = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

            //rotates the stick values by the robot's heading
            double rotatedX = (stickPos.x*Math.cos(-heading))-(stickPos.y*Math.sin(-heading));
            double rotatedY = (stickPos.x*Math.sin(-heading))+(stickPos.y*Math.cos(-heading));

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            rotatedX,
                            rotatedY
                    ),
                    -gamepad1.right_stick_x
            ));

            // reset robot orientation
            if (gamepad1.back){
                if (gamepad1.y){
                    robotOffset = 0;
                    drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);
                } else if (gamepad1.b){
                    robotOffset = Math.toRadians(-90);
                    drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);
                } else if (gamepad1.a){
                    robotOffset = Math.toRadians(180);
                    drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);
                } else if (gamepad1.x){
                    robotOffset = Math.toRadians(90);
                    drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);
                }
            }

//            telemetry.addData("x", drive.pose.position.x);
//            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}

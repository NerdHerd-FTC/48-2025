package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
@Config
public class mecanumDriveFO extends LinearOpMode {

    public boolean intakeState = true;
    //TODO: change the first index to the top basket position
    public static double[] outtakePositions = {0.0,0.51,0.515,0.8,0.84,1.0};
    public int outtakePos = 0;

    @Override
    public void runOpMode() {
        // import drive and slides
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        linearSlides arm = new linearSlides();

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
        CRServo intakeCactusL = hardwareMap.get(CRServo.class,"intakeCactusL");
        CRServo intakeCactusR = hardwareMap.get(CRServo.class,"intakeCactusR");
        Servo outtakeSpin = hardwareMap.get(Servo.class,"outtakeSpin");


        // set target position for RUN_TO_POSITION
//        outtakeSlideL.setTargetPosition(outtakeSlideL.getCurrentPosition());
//        outtakeSlideR.setTargetPosition(outtakeSlideR.getCurrentPosition());

        // set mode
        outtakeSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set velocity for RUN_TO_POSITION
//        outtakeSlideL.setVelocity(org.firstinspires.ftc.teamcode.teleop.linearSlides.SLIDE_MAX_VEL);
//        outtakeSlideR.setVelocity(org.firstinspires.ftc.teamcode.teleop.linearSlides.SLIDE_MAX_VEL);

        // set ZeroPowerBehavior
        outtakeSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set directions
        // TODO: set these correctly
        intakeSlideL.setDirection(Servo.Direction.REVERSE);
        outtakePivotL.setDirection(Servo.Direction.REVERSE);
        intakeTopPivotR.setDirection(Servo.Direction.REVERSE);
        outtakeSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeCactusR.setDirection(DcMotorSimple.Direction.REVERSE);



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

            telemetry.addLine("Press BACK to reset outtake extension positions");

            if (gamepad1.back){
                outtakeSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtakeSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                outtakeSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outtakeSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            telemetry.addData("Robot Offset", robotOffset);
            telemetry.addData("Robot Direction", robotDirection);
            telemetry.addData("Left Outtake Slide Position", outtakeSlideL.getCurrentPosition());
            telemetry.addData("Right Outtake Slide Position", outtakeSlideR.getCurrentPosition());
            telemetry.addLine("ready to go");
            telemetry.update();
        }
        waitForStart();

        outtakeClaw.setPosition(arm.outtakeClawCalc(0.0));
        boolean outtakeClawOpen = true;
        boolean aPrevPos = gamepad1.a;
        boolean bPrevPos = gamepad1.b;
        boolean yPrevPos = gamepad1.y;
        boolean xPrevPos = gamepad1.x;
        boolean rbPrevPos = gamepad1.right_bumper;
        boolean lbPrevPos = gamepad1.left_bumper;

        outtakePivotL.setPosition(arm.outtakePivotCalc(outtakePositions[outtakePos]));
        outtakePivotR.setPosition(arm.outtakePivotCalc(outtakePositions[outtakePos]));

        intakeSlideL.setPosition(arm.intakeExtendCalc(0.0));
        intakeSlideR.setPosition(arm.intakeExtendCalc(0.0));

        intakeTopPivotL.setPosition(arm.intakeTopPivotCalc(1.0));
        intakeTopPivotR.setPosition(arm.intakeTopPivotCalc(1.0));

        // Outtake Spinner Block
        outtakeSpin.setPosition(arm.outtakeSpinCalc(0.0));

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

            // Outtake Slide Block
            double slidePower = (-gamepad1.left_trigger) + gamepad1.right_trigger;
            if ((outtakeSlideL.getCurrentPosition() < arm.outakeExtendCalc(1.0)) && (outtakeSlideR.getCurrentPosition() < arm.outakeExtendCalc(1.0)) && (slidePower > 0.0)){
                outtakeSlideL.setPower(slidePower);
                outtakeSlideR.setPower(slidePower);
            } else if ((outtakeSlideL.getCurrentPosition() > arm.outakeExtendCalc(0.0)) && (outtakeSlideR.getCurrentPosition() > arm.outakeExtendCalc(0.0)) && (slidePower < 0.0)){
                outtakeSlideL.setPower(slidePower);
                outtakeSlideR.setPower(slidePower);
            } else {
                outtakeSlideL.setPower(0);
                outtakeSlideR.setPower(0);
            }
            telemetry.addData("Outtake Slide Input", slidePower);
            telemetry.addData("Outtake Slide L Position", outtakeSlideL.getCurrentPosition());
            telemetry.addData("Outtake Slide R Position", outtakeSlideR.getCurrentPosition());

            // Outtake Pivot Block
            if (!gamepad1.back) {
                if (gamepad1.x && !xPrevPos && (outtakePos < outtakePositions.length-1)) {
                    outtakePos++;
                } else if (gamepad1.y && !yPrevPos && (outtakePos > 0)) {
                    outtakePos--;
                }
            }

            if (outtakePos == 0){
                // Outtake Spinner Block
                outtakeSpin.setPosition(arm.outtakeSpinCalc(0.0));
            } else {
                // Outtake Spinner Block
                outtakeSpin.setPosition(arm.outtakeSpinCalc(1.0));
            }

            outtakePivotL.setPosition(arm.outtakePivotCalc(outtakePositions[outtakePos]));
            outtakePivotR.setPosition(arm.outtakePivotCalc(outtakePositions[outtakePos]));

            telemetry.addData("Outtake Pivot Position", outtakePos);


            // Outtake Claw Block
            if ((!aPrevPos) && (gamepad1.a) && (!gamepad1.back)){
                if (outtakeClawOpen){
                    outtakeClaw.setPosition(arm.outtakeClawCalc(1.0));
                } else {
                    outtakeClaw.setPosition(arm.outtakeClawCalc(0.0));
                }
                outtakeClawOpen = !outtakeClawOpen;
            }
            telemetry.addData("Is Outtake Claw Open?", outtakeClawOpen);

//            // Intake Slide Block
//            if (gamepad1.dpad_up && !gamepad1.dpad_down){
//                intakeSlideL.setPosition(arm.intakeExtendCalc(1.0));
//                intakeSlideR.setPosition(arm.intakeExtendCalc(1.0));
//            }
            if (!gamepad1.dpad_up && gamepad1.dpad_down){
                intakeSlideL.setPosition(arm.intakeExtendCalc(0.0));
                intakeSlideR.setPosition(arm.intakeExtendCalc(0.0));
            }

            // Intake Pivot Block
            if (gamepad1.right_bumper && !rbPrevPos){
                intakeTopPivotL.setPosition(arm.intakeTopPivotCalc(0));
                intakeTopPivotR.setPosition(arm.intakeTopPivotCalc(0));
            }
            if (gamepad1.left_bumper && !lbPrevPos){
                intakeTopPivotL.setPosition(arm.intakeTopPivotCalc(1.0));
                intakeTopPivotR.setPosition(arm.intakeTopPivotCalc(1.0));
            }

            // Intake Cactus Block
            // true is intaking
            if ((!bPrevPos) && (gamepad1.b) && (!gamepad1.back)){
                if (intakeCactusL.getPower() == 0.0) {
                    if (intakeState) {
                        intakeCactusL.setPower(0.5);
                        intakeCactusR.setPower(0.5);
                    } else {
                        intakeCactusL.setPower(-0.5);
                        intakeCactusR.setPower(-0.5);
                    }
                    intakeState = !intakeState;
                } else{
                    intakeCactusL.setPower(0.0);
                    intakeCactusR.setPower(0.0);
                }
            }
            telemetry.addData("Is Intake On", intakeState);



            aPrevPos = gamepad1.a;
            bPrevPos = gamepad1.b;
            yPrevPos = gamepad1.y;
            xPrevPos = gamepad1.x;
            rbPrevPos = gamepad1.right_bumper;
            lbPrevPos = gamepad1.left_bumper;

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine("im wheler and i love duenes");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}

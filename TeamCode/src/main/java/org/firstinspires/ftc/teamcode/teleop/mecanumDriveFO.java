package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="FO Mecanum Drive")
public class mecanumDriveFO extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        swingArm armControl = new swingArm();

        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        DcMotorEx actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        Servo pivot = hardwareMap.get(Servo.class, "pivot");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        actuator.setTargetPosition(0);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setVelocity(armControl.ARM_MAX_VEL);
        actuator.setVelocity(armControl.ACTUATOR_MAX_VEL);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        actuator.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);


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


            telemetry.addData("Robot Offset", robotOffset);
            telemetry.addData("Robot Direction", robotDirection);
            telemetry.addLine("ready to go");
            telemetry.update();
        }
        waitForStart();

        double pivotPos = 0.5;

        drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);

        while (opModeIsActive()){

            if (gamepad1.left_bumper & !gamepad1.right_bumper){
                armControl.moveClaw(0.0,claw);
            } else if (gamepad1.right_bumper){
                armControl.moveClaw(1.0,claw);
            }

            telemetry.addData("Claw Position", claw.getPosition());

            if (gamepad1.dpad_up ^ gamepad1.dpad_down ^ gamepad1.dpad_right ^ gamepad1.dpad_left) {
                //on ground
                if (gamepad1.dpad_down) {
                    armControl.moveArm(0.0, arm);
                }
                //halfway
                if (gamepad1.dpad_right){
                    armControl.moveArm(0.5,arm);
                }
                if (gamepad1.dpad_left){
                    armControl.moveArm(0.85,arm);
                }
                //slightly backwards
                if (gamepad1.dpad_up) {
                    armControl.moveArm(1.0, arm);
                }
            }

            telemetry.addData("Arm Position", arm.getCurrentPosition());

            pivotPos = pivotPos + (gamepad1.right_trigger * .10) - (gamepad1.left_trigger * .10);

            //keeps pivotPos between 0 and 1
            pivotPos = (pivotPos<0) ? 0 : (pivotPos>1) ? 1 : pivotPos;

            armControl.clawPivot(pivotPos,pivot);

            telemetry.addData("Pivot Position", pivot.getPosition());

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

            if (gamepad1.back){
                if (gamepad1.y){
                    robotOffset = 0;
                    robotDirection = "Forward";
                    drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);
                } else if (gamepad1.b){
                    robotOffset = Math.toRadians(-90);
                    robotDirection = "Right";
                    drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);
                } else if (gamepad1.a){
                    robotOffset = Math.toRadians(180);
                    robotDirection = "Reverse";
                    drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);
                } else if (gamepad1.x){
                    robotOffset = Math.toRadians(90);
                    robotDirection = "Left";
                    drive.pose = new Pose2d(new Vector2d(0,0),robotOffset);
                }
            } else if (gamepad1.a ^ gamepad1.b ^ gamepad1.x ^ gamepad1.y){
                //fully retracted
                if (gamepad1.a){
                    armControl.moveActuator(0.0,actuator);
                }
                //half extended
                if (gamepad1.b || gamepad1.x){
                    armControl.moveActuator(0.5,actuator);
                }
                //fully extended
                if (gamepad1.y){
                    armControl.moveActuator(1.0,actuator);
                }
            }

            telemetry.addData("Actuator Position", actuator.getCurrentPosition());

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}

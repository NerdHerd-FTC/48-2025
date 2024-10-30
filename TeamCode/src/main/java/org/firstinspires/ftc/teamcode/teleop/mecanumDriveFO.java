package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="Mecanum Drive FO")
public class mecanumDriveFO extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        double driverDirection = 0;
        String driverLocation = "Audience";
        double robotOffset = 0;
        double robotHeading = 0;
        String robotDirection = "Forward";
        telemetry.addData("Driver Location", driverLocation);
        telemetry.addData("Driver Direction", driverDirection);
        telemetry.addData("Robot Offset", robotOffset);
        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Robot Direction", robotDirection);

        while (opModeInInit()) {
            telemetry.addLine("Press the D-Pad Button towards the audience");

            if (gamepad1.dpad_down){
                driverDirection = 0;
                driverLocation = "Audience";
            } else if (gamepad1.dpad_left) {
                driverDirection = Math.toRadians(-90);
                driverLocation = "Red Alliance";
            } else if (gamepad1.dpad_up) {
                driverDirection = Math.toRadians(180);
                driverLocation = "Scoring Table";
            } else if (gamepad1.dpad_right) {
                driverDirection = Math.toRadians(90);
                driverLocation = "Blue Alliance";
            }

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

            robotHeading = driverDirection + robotOffset;

            telemetry.addData("Driver Location", driverLocation);
            telemetry.addData("Driver Direction", driverDirection);
            telemetry.addData("Robot Offset", robotOffset);
            telemetry.addData("Robot Heading", robotHeading);
            telemetry.addData("Robot Direction", robotDirection);
            telemetry.addLine("ready to go");
            telemetry.update();
        }
        waitForStart();

        drive.pose = new Pose2d(new Vector2d(0,0),robotHeading);

        while (opModeIsActive()){
            drive.updatePoseEstimate();

            double heading = drive.pose.heading.toDouble()+driverDirection;

            //take a look at an explanation of this math at https://www.desmos.com/calculator/yxpg9zuzt4
            //these values are flipped due to the rotation of roadrunner's coordinate system
            Vector2d stickPos = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

            //rotates the stick values by the robot's heading
            double rotatedX = (stickPos.x*Math.cos(heading))-(stickPos.y*Math.sin(heading));
            double rotatedY = (stickPos.x*Math.sin(heading))+(stickPos.y*Math.cos(heading));

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            rotatedX,
                            rotatedY
                    ),
                    -gamepad1.right_stick_x
            ));

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

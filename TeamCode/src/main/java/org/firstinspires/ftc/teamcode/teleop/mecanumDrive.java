package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Mecanum Drive")
public class mecanumDrive extends LinearOpMode {
    public void runOpMode() {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Turn off encoders for drivetrain
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //change these if the motors don't spin the correct directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("ready to go");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive(frontLeft,frontRight,backLeft,backRight,1.0, gamepad1);
            telemetry.addLine("yippee! the robot is working!");
            telemetry.update();
        }

    }

    public void drive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, double strafe_speed, Gamepad gamepad1) {
        // get values from controller
        double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
        double stickX = gamepad1.left_stick_x*strafe_speed;
        double rStickX = gamepad1.right_stick_x;

        // get denominator
        double denominator = Math.max(Math.abs(stickX) + Math.abs(stickY) + Math.abs(rStickX), 1);
        // denominator ensures ratios are maintained, because the motors only go from 0-1

        // set values based on mecanum drive
        frontLeft.setPower((stickY + stickX+ rStickX) / denominator);
        frontRight.setPower((stickY - stickX - rStickX) / denominator);
        backLeft.setPower((stickY - stickX + rStickX) / denominator);
        backRight.setPower((stickY + stickX - rStickX) / denominator);
    }
}

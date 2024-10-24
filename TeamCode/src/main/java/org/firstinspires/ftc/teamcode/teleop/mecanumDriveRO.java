package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Mecanum Drive RO")
public class mecanumDriveRO extends LinearOpMode {
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        // Turn off encoders for drivetrain
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: set these
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("ready to go");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive(frontLeft,frontRight,backLeft,backRight,gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x,1.0);

            telemetry.addLine("yippee! the robot is working!");
            telemetry.update();
        }

    }

    public void drive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, double stickY, double stickX, double rStickX, double strafe_speed) {
        stickY = -stickY; //Y stick value is REVERSED
        stickX = stickX*strafe_speed;
        //rStickY value is the same

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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.mecanumDriveRO;
import org.firstinspires.ftc.teamcode.teleop.swingArm;

@TeleOp(name="OLD RO Mecanum Drive with Arm")
@Disabled
public class swingArmDrive extends LinearOpMode {
    public void runOpMode() {
        mecanumDriveRO drive = new mecanumDriveRO();
        swingArm armControl = new swingArm();

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        DcMotorEx actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        Servo pivot = hardwareMap.get(Servo.class, "pivot");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        actuator.setTargetPosition(0);

        // Turn off encoders for drivetrain
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setVelocity(armControl.ARM_MAX_VEL);
        actuator.setVelocity(armControl.ACTUATOR_MAX_VEL);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //change these if the motors don't spin the correct directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        actuator.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("ready to go");
        telemetry.addLine("im wheler and i love duenes");
        telemetry.update();

        waitForStart();

        double pivotPos = 0.5;

        while (opModeIsActive()) {
            drive.drive(frontLeft,frontRight,backLeft,backRight, gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x,1.0);

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

            if (gamepad1.a ^ gamepad1.b ^ gamepad1.x ^ gamepad1.y){
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

            pivotPos = pivotPos + (gamepad1.right_trigger * .10) - (gamepad1.left_trigger * .10);

            //keeps pivotPos between 0 and 1
            pivotPos = (pivotPos<0) ? 0 : (pivotPos>1) ? 1 : pivotPos;

            armControl.clawPivot(pivotPos,pivot);

            telemetry.addData("Pivot Position", pivot.getPosition());

            telemetry.addLine("yippee! the robot is working!");
            telemetry.update();
        }

    }
}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Swing Arm")
public class swingArm extends LinearOpMode {

    //TODO: set these to be reasonable speeds
    //these values are in ticks per second
    //formatted as (desired rpm/60) * (motor ticks per rotation)
    public static final double ARM_MAX_VEL = (117.0/60) * 1425.1; //117 rpm: 1425.1 ticks per rotation
    public static final double ACTUATOR_MAX_VEL = (335/60.0) * 384.5; //335 rpm: 384.5 ticks per rotation

    @Override
    public void runOpMode(){
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

        arm.setVelocity(ARM_MAX_VEL);
        actuator.setVelocity(ACTUATOR_MAX_VEL);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: Set this uwu
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        actuator.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("ready to go");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Arm","Gamepad 1 D-Pad");
            if (gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right){
                moveArm(1.0, arm);
            }
            if ((gamepad1.dpad_left || gamepad1.dpad_right) && !gamepad1.dpad_up && !gamepad1.dpad_down){
                moveArm(0.5, arm);
            }
            if (gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_left && !gamepad1.dpad_right){
                moveArm(0.0, arm);
            }
            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            telemetry.addData("Arm Target", arm.getTargetPosition());

            telemetry.addData("Actuator", "Gamepad 1 Face Buttons");
            if (gamepad1.y && !gamepad1.b && !gamepad1.x && !gamepad1.a){
                moveActuator(1.0, actuator);
            }
            if ((gamepad1.x || gamepad1.b) && !gamepad1.a && !gamepad1.y){
                moveActuator(0.5, actuator);
            }
            if (gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y){
                moveActuator(0.0, actuator);
            }
            telemetry.addData("Actuator Pos", actuator.getCurrentPosition());
            telemetry.addData("Actuator Target", actuator.getTargetPosition());

            telemetry.addData("Pivot", "Gamepad 2 D-Pad");
            if (gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad1.dpad_left && !gamepad2.dpad_right){
                clawPivot(1.0, pivot);
            }
            if ((gamepad2.dpad_left || gamepad2.dpad_right) && !gamepad2.dpad_up && !gamepad2.dpad_down){
                clawPivot(0.5, pivot);
            }
            if (gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right){
                clawPivot(0.0, pivot);
            }
            telemetry.addData("Pivot Pos", pivot.getPosition());

            telemetry.addData("Claw","Gamepad 2 Face Buttons");
            if (gamepad2.y && !gamepad2.b && !gamepad2.x && !gamepad2.a) {
                moveClaw(1.0, claw);
            }
            if ((gamepad2.x || gamepad2.b) && !gamepad2.a && !gamepad2.y){
                moveClaw(0.5, claw);
            }
            if (gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y){
                moveClaw(0.0, claw);
            }
            telemetry.addData("Claw Pos", claw.getPosition());

            telemetry.addLine("yippee! the robot is working!");
            telemetry.update();
        }
    }

    public void moveArm(double pos, DcMotor arm){
        final int UPPER_BOUND = 2900;

        int targetPosition = (int) (pos * UPPER_BOUND);

        arm.setTargetPosition(targetPosition);
    }

    public void moveActuator(double pos, DcMotor arm){
        final int UPPER_BOUND = 9038;

        int targetPosition = (int) (pos * UPPER_BOUND);

        arm.setTargetPosition(targetPosition);
    }

    public void clawPivot(double pos, Servo pivot){
        final double LOWER_BOUND = 0.3;
        final double UPPER_BOUND = 0.7;

        double targetPosition = (pos*(UPPER_BOUND-LOWER_BOUND)) + LOWER_BOUND;

        pivot.setPosition(targetPosition);
    }

    public void moveClaw(double pos, Servo claw){
        final double LOWER_BOUND = 0.05;
        final double UPPER_BOUND = 0.55;

        double targetPosition = (pos*(UPPER_BOUND-LOWER_BOUND)) +LOWER_BOUND;

        claw.setPosition(targetPosition);
    }
}


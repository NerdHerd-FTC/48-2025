package org.firstinspires.ftc.teamcode.teleop;

import android.icu.text.CaseMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Swing Arm")
public class swingArm extends LinearOpMode {

    @Override
    public void runOpMode(){
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor actuator = hardwareMap.dcMotor.get("actuator");
        Servo pivotL = hardwareMap.servo.get("pivotL");
        Servo pivotR = hardwareMap.servo.get("pivotL");
        Servo claw = hardwareMap.servo.get("pivotL");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: Set this uwu
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotL.setDirection(Servo.Direction.FORWARD);
        pivotR.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("ready to go");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right){
                moveArm(1.0, arm);
            }
            if ((gamepad1.dpad_left || gamepad1.dpad_right) && !gamepad1.dpad_up && !gamepad1.dpad_down){
                moveArm(0.5, arm);
            }
            if (gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_left && !gamepad1.dpad_right){
                moveArm(0.0, arm);
            }

            if (gamepad1.y && !gamepad1.b && !gamepad1.x && !gamepad1.a){
                moveActuator(1.0, actuator);
            }
            if ((gamepad1.x || gamepad1.b) && !gamepad1.a && !gamepad1.y){
                moveActuator(0.5, actuator);
            }
            if (gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y){
                moveActuator(0.0, actuator);
            }

            if (gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad1.dpad_left && !gamepad2.dpad_right){
                clawPivot(1.0, pivotL, pivotR);
            }
            if ((gamepad2.dpad_left || gamepad2.dpad_right) && !gamepad2.dpad_up && !gamepad2.dpad_down){
                clawPivot(0.5, pivotL, pivotR);
            }
            if (gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right){
                clawPivot(0.0, pivotL, pivotR);
            }

            if (gamepad2.y && !gamepad2.b && gamepad2.x && !gamepad2.a) {
                moveClaw(0.1, claw);
            }
            if ((gamepad2.x || gamepad2.b) && !gamepad2.a && !gamepad2.y){
                moveClaw(0.5, claw);
            }
            if (gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y){
                moveClaw(0.0, claw);
            }




            telemetry.addLine("yippee! the robot is working!");
            telemetry.update();
        }
    }

    public void moveArm(double pos, DcMotor arm){
        final int UPPER_BOUND = 300;

        int targetPosition = (int) (pos * UPPER_BOUND);

        arm.setTargetPosition(targetPosition);
    }

    public void moveActuator(double pos, DcMotor arm){
        final int UPPER_BOUND = 300;

        int targetPosition = (int) (pos * UPPER_BOUND);

        arm.setTargetPosition(targetPosition);
    }

    public void clawPivot(double pos, Servo pivotL, Servo pivotR){
        final double LOWER_BOUND = 0.2;
        final double UPPER_BOUND = 0.8;

        double targetPosition = (pos*(UPPER_BOUND-LOWER_BOUND)) + LOWER_BOUND;

        pivotL.setPosition(targetPosition);
        pivotR.setPosition(targetPosition);
    }

    public void moveClaw(double pos, Servo Claw){
        final double LOWER_BOUND = 0.2;
        final double UPPER_BOUND = 0.8;

        double targetPosition = (pos*(UPPER_BOUND-LOWER_BOUND)) +LOWER_BOUND;

        Claw.setPosition(targetPosition);

    }
}


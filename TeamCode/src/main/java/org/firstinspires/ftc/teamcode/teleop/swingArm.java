package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Swing Arm")
public class swingArm extends LinearOpMode {

    @Override
    public void runOpMode(){
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor actuator = hardwareMap.dcMotor.get("actuator");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: Set this uwu
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

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
}

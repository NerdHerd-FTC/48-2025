package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Unextend Arm")
@Disabled
public class unextendArm extends LinearOpMode {
    public void runOpMode(){
        DcMotorEx actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setTargetPosition(0);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setVelocity((20/60.0) * 384.5);
        actuator.setDirection(DcMotorSimple.Direction.FORWARD);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("ready to go");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down){
                actuator.setTargetPosition(actuator.getTargetPosition()-10);
            }
            telemetry.addLine("Press D-Pad down to move 10 ticks down");
            telemetry.addData("Acutator Position", actuator.getCurrentPosition());
            telemetry.addData("Actuator Target", actuator.getTargetPosition());
            telemetry.update();
        }
    }
}

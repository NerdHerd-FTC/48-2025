package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Motor Maker Mover", group="yzz")
@Config
public class makeMotorSpinPlease extends LinearOpMode {
    public static String MOTOR_NAME = "spinThisMotor";
    public static double MOTOR_POWER = 0.1;

    @Override
    public void runOpMode() throws InterruptedException{
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        telemetry.addData("Motor Name", MOTOR_NAME);
        telemetry.addData("Motor Power", MOTOR_POWER);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            motor.setPower(MOTOR_POWER);
            telemetry.addLine("Yippee!");
            telemetry.update();
        }
    }
}

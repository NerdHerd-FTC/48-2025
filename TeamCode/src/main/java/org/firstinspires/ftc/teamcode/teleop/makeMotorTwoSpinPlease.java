package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Dual Motor Maker Mover", group="yzz")
@Config
public class makeMotorTwoSpinPlease extends LinearOpMode {
    public static String MOTOR1_NAME = "spinThisMotor";
    public static String MOTOR2_NAME = "spinThisMotor";
    public static double MOTOR_POWER = 0.1;
    public static boolean FLIP_MOTOR_1 = false;
    public static boolean FLIP_MOTOR_2 = true;

    @Override
    public void runOpMode() throws InterruptedException{
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, MOTOR1_NAME);
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, MOTOR2_NAME);

        if (FLIP_MOTOR_1){
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (FLIP_MOTOR_2){
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        telemetry.addData("Motor 1 Name", MOTOR1_NAME);
        telemetry.addData("Motor 1 Flip", FLIP_MOTOR_1);
        telemetry.addData("Motor 2 Name", MOTOR2_NAME);
        telemetry.addData("Motor 2 Flip", FLIP_MOTOR_2);
        telemetry.addData("Motor Power", MOTOR_POWER);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            motor1.setPower(MOTOR_POWER);
            motor2.setPower(MOTOR_POWER);
            telemetry.addData("Motor 1 Pos", motor1.getCurrentPosition());
            telemetry.addData("Motor 2 Pos", motor2.getCurrentPosition());
            telemetry.addLine("Yippee!");
            telemetry.update();
        }
    }
}

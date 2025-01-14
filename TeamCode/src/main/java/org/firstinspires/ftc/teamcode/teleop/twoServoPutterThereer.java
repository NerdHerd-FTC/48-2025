package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Dual Servo Putter Thereer", group="yzz")
@Config
public class twoServoPutterThereer extends LinearOpMode {
    public static double TARGET_POSITION = 0.5;
    public static String SERVO1 = "intakeSlideR";
    public static String SERVO2 = "intakeSlideL";
    public static boolean FLIP_SERVO_1 = false;
    public static boolean FLIP_SERVO_2 = true;
    @Override
    public void runOpMode() throws InterruptedException{
        Servo servo1 = hardwareMap.get(Servo.class, SERVO1);
        Servo servo2 = hardwareMap.get(Servo.class, SERVO2);

        if (FLIP_SERVO_1){
            servo1.setDirection(Servo.Direction.REVERSE);
        }
        if (FLIP_SERVO_2){
            servo2.setDirection(Servo.Direction.REVERSE);
        }

        telemetry.addData("Target Position", TARGET_POSITION);
        telemetry.addData("Servo 1", SERVO1);
        telemetry.addData("Servo 1 Flip", FLIP_SERVO_1);
        telemetry.addData("Servo 2", SERVO2);
        telemetry.addData("Servo 2 Flip", FLIP_SERVO_2);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            servo1.setPosition(TARGET_POSITION);
            servo2.setPosition(TARGET_POSITION);
            telemetry.addLine("Yippee!");
            telemetry.addData("Time Elapsed", getRuntime());
            telemetry.update();
        }
    }
}

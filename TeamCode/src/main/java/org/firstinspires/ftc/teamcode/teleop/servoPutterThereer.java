package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Putter Thereer", group="yzz")
@Config
public class servoPutterThereer extends LinearOpMode {
    public static String SERVO = "putThisServo";
    public static double TARGET_POSITION = 0.5;

    @Override
    public void runOpMode() throws InterruptedException{
        Servo servo = hardwareMap.get(Servo.class,SERVO);

        telemetry.addData("Target Position", TARGET_POSITION);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            servo.setPosition(TARGET_POSITION);
            telemetry.addLine("Yippee!");
            telemetry.addData("Time Elapsed", getRuntime());
            telemetry.update();
        }
    }
}

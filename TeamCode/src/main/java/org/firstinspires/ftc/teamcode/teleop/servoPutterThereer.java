package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Putter Thereer")
public class servoPutterThereer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Servo servo = hardwareMap.get(Servo.class, "putThisServo");

        final double TARGET_POSITION = 0.5;

        telemetry.addData("Target Position", TARGET_POSITION);
        telemetry.update();

        waitForStart();

        servo.setPosition(TARGET_POSITION);
        while (opModeIsActive()){
            telemetry.addLine("Yippee!");
            telemetry.addData("Time Elapsed", getRuntime());
            telemetry.update();
        }
    }
}

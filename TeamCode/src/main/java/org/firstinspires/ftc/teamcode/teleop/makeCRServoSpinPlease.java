package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="CRServo Spinner", group="yzz")
@Config
public class makeCRServoSpinPlease extends LinearOpMode {
    public static String SERVO = "intakeCactusL";
    public static double POWER = 0.1;

    @Override
    public void runOpMode() throws InterruptedException{
        CRServo servo = hardwareMap.get(CRServo.class,SERVO);

        telemetry.addData("POWER", POWER);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            servo.setPower(POWER);
            telemetry.addLine("Yippee!");
            telemetry.addData("Time Elapsed", getRuntime());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//TODO: change this to the mecanum drive OpMode
@Autonomous(name="Do Nothing", preselectTeleOp = "Mecanum Drive")
public class doNothing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine("im wheler and i love duenes");
            telemetry.update();
        }
    }
}

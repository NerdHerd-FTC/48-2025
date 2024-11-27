package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="Slide Arm")
public class slideArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }

    public void moveSlide(double pos, DcMotor armL, DcMotor armR){
        final int UPPER_BOUND = 200;

        int targetPosition = (int) (pos * UPPER_BOUND);

        armL.setTargetPosition(targetPosition);
        armR.setTargetPosition(targetPosition);
    }
}

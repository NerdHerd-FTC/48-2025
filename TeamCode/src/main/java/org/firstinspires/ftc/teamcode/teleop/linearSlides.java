package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Linear Slide Controls")
@Disabled
public class linearSlides extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("no");
        telemetry.update();
        waitForStart();
    }

    public double intakeExtendCalc(double pos){
        final double BOTTOM = 0.4;
        final double TOP = 0.6;

        // Multiply position by the range available,
        // then add the bottom number
        return (pos * (TOP - BOTTOM)) + BOTTOM;
    }

    public int outakeExtendCalc(double pos){
        final int BOTTOM = 0;
        final int TOP = 100;

        return (int) ((pos * (TOP - BOTTOM)) + BOTTOM);
    }

    public double intakeTopPivotCalc(double pos){
        final double BOTTOM = 0.4;
        final double TOP = 0.6;

        return (pos * (TOP - BOTTOM)) + BOTTOM;
    }

    public double intakeBottomPivotCalc(double pos){
        final double BOTTOM = 0.4;
        final double TOP = 0.6;

        return ((pos * TOP - BOTTOM)) + BOTTOM;
    }

    public double outtakePivotCalc(double pos){
        final double BOTTOM = 0.4;
        final double TOP = 0.6;

        return ((pos * TOP - BOTTOM)) + BOTTOM;
    }

    public double outtakeClawCalc(double pos){
        final double BOTTOM = 0.4;
        final double TOP = 0.6;

        return ((pos * TOP - BOTTOM)) + BOTTOM;
    }
}

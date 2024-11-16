package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.swingArm;

@Config
@Disabled
@Autonomous(name="Swing Arm Actions")
public class swingArmActions extends LinearOpMode {
    private swingArm armControl = new swingArm();

    public void runOpMode() throws InterruptedException{
        telemetry.addLine("what are you doing");
        telemetry.update();
        waitForStart();
    }

    public class Swing {
        private DcMotorEx arm;

        public Swing(HardwareMap hardwareMap){
            arm = hardwareMap.get(DcMotorEx.class,"arm");
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(armControl.ARM_MAX_VEL);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class MoveSwingScore implements Action {
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                armControl.moveArm(0.85,arm);
                return false;
            }
        }

        public Action moveSwingScore(){
            return new MoveSwingScore();
        }

        public class MoveSwingDown implements Action{
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!init) {
                    armControl.moveArm(0, arm);
                }
                if (arm.getCurrentPosition() > 0){
                    return true;
                }
                return false;
            }
        }

        public Action moveSwingDown(){
            return new MoveSwingDown();
        }
    }

    public class Actuator {
        private DcMotorEx actuator;

        public Actuator(HardwareMap hardwareMap){
            actuator = hardwareMap.get(DcMotorEx.class,"actuator");
            actuator.setDirection(DcMotorSimple.Direction.FORWARD);
            actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            actuator.setTargetPosition(0);
            actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            actuator.setVelocity(armControl.ACTUATOR_MAX_VEL);
            actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class MoveActuatorScore implements Action {
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!init) {
                    armControl.moveActuator(1, actuator);
                }
                if (actuator.getCurrentPosition() < 9038 - 100){
                    return true;
                }

                return false;
            }
        }

        public Action moveActuatorScore(){
            return new MoveActuatorScore();
        }

        public class MoveActuatorDown implements Action{
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!init) {
                    armControl.moveActuator(0, actuator);
                }
                if (actuator.getCurrentPosition() > 10){
                    return true;
                }

                return false;
            }
        }

        public Action moveActuatorDown(){
            return new MoveActuatorDown();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class,"claw");
            claw.setDirection(Servo.Direction.FORWARD);
        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                armControl.moveClaw(0,claw);
                return false;
            }
        }

        public Action openClaw(){
            return new OpenClaw();
        }

        public class CloseClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                armControl.moveClaw(1,claw);
                return false;
            }
        }

        public Action closeClaw(){
            return new CloseClaw();
        }
    }

    public class Pivot {
        private Servo pivot;

        public Pivot(HardwareMap hardwareMap){
            pivot = hardwareMap.get(Servo.class,"pivot");
            pivot.setDirection(Servo.Direction.FORWARD);
        }

        public class PivotDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                armControl.clawPivot(-0.1,pivot);
                return false;
            }
        }

        public Action pivotDown(){
            return new PivotDown();
        }

        public class PivotUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                armControl.clawPivot(0.5,pivot);
                return false;
            }
        }

        public Action pivotUp(){
            return new PivotUp();
        }
    }
}

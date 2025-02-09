package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.message.redux.StartOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.linearSlides;

@Config
@Autonomous(name="Linear Slides Actions")
@Disabled
public class linearSlidesActions extends LinearOpMode {
    private linearSlides armControl = new linearSlides();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addLine("what are you doing");
        telemetry.update();
        waitForStart();
    }

    public class IntakeCactus {
        private CRServo intakeCactusL;
        private CRServo intakeCactusR;

        public IntakeCactus() {
            intakeCactusL = hardwareMap.get(CRServo.class,"intakeCactusL");
            intakeCactusR = hardwareMap.get(CRServo.class,"intakeCactusR");
            intakeCactusR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class SpinIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakeCactusL.setPower(1);
                intakeCactusR.setPower(1);
                return false;
            }
        }

        public Action spinIn(){
            return new SpinIn();
        }

        public class SpinOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakeCactusL.setPower(-1);
                intakeCactusR.setPower(-1);
                return false;
            }
        }

        public Action spinOut(){
            return new SpinOut();
        }

        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakeCactusL.setPower(0);
                intakeCactusR.setPower(0);
                return false;
            }
        }

        public Action stop(){
            return new Stop();
        }
    }

    public class IntakePivot{
        private Servo intakePivotL;
        private Servo intakePivotR;

        public IntakePivot(){
            intakePivotL = hardwareMap.get(Servo.class,"intakeTopPivotL");
            intakePivotR = hardwareMap.get(Servo.class,"intakeTopPivotR");
            intakePivotR.setDirection(Servo.Direction.REVERSE);
        }

        public class Down implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakePivotL.setPosition(armControl.intakeTopPivotCalc(0));
                intakePivotR.setPosition(armControl.intakeTopPivotCalc(0));
                return false;
            }
        }

        public Action down(){
            return new Down();
        }

        public class Up implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakePivotL.setPosition(armControl.intakeTopPivotCalc(1));
                intakePivotR.setPosition(armControl.intakeTopPivotCalc(1));
                return false;
            }
        }

        public Action up(){
            return new Up();
        }
    }

    public class OuttakeClaw{
        private Servo outtakeClaw;

        public OuttakeClaw(){
            outtakeClaw = hardwareMap.get(Servo.class,"outtakeClaw");
        }

        public class Open implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                outtakeClaw.setPosition(armControl.outtakeClawCalc(0));
                return false;
            }
        }

        public Action open(){
            return new Open();
        }

        public class Close implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                outtakeClaw.setPosition(armControl.outtakeClawCalc(1));
                return false;
            }
        }

        public Action close(){
            return new Close();
        }
    }

    public class IntakeSlide{
        private Servo intakeSlideL;
        private Servo intakeSlideR;

        public IntakeSlide(){
            intakeSlideL = hardwareMap.get(Servo.class,"intakeSlideL");
            intakeSlideR = hardwareMap.get(Servo.class,"intakeSlideR");
            intakeSlideL.setDirection(Servo.Direction.REVERSE);
        }

        public class Out implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakeSlideL.setPosition(armControl.intakeExtendCalc(1));
                intakeSlideR.setPosition(armControl.intakeExtendCalc(1));
                return false;
            }
        }

        public Action out(){
            return new Out();
        }

        public class In implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intakeSlideL.setPosition(armControl.intakeExtendCalc(0));
                intakeSlideR.setPosition(armControl.intakeExtendCalc(0));
                return false;
            }
        }

        public Action in(){
            return new In();
        }
    }

    public class OuttakePivot{
        private Servo outtakePivotL;
        private Servo outtakePivotR;
        private double[] outtakePositions;

        public OuttakePivot(){
            outtakePivotL = hardwareMap.get(Servo.class,"outtakePivotL");
            outtakePivotR = hardwareMap.get(Servo.class,"outtakePivotR");
            outtakePivotL.setDirection(Servo.Direction.REVERSE);
            outtakePositions = org.firstinspires.ftc.teamcode.teleop.mecanumDriveFO.outtakePositions;
        }

        public class Floor implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                outtakePivotL.setPosition(armControl.outtakePivotCalc(1.0));
                outtakePivotR.setPosition(armControl.outtakePivotCalc(1.0));
                return false;
            }
        }
        public Action floor(){
            return new Floor();
        }

        public class Score implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                outtakePivotL.setPosition(armControl.outtakePivotCalc(outtakePositions[1]));
                outtakePivotR.setPosition(armControl.outtakePivotCalc(outtakePositions[1]));
                return false;
            }
        }
        public Action score(){
            return new Score();
        }

        public class Intake implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                outtakePivotL.setPosition(armControl.outtakePivotCalc(0));
                outtakePivotR.setPosition(armControl.outtakePivotCalc(0));
                return false;
            }
        }
        public Action intake(){
            return new Intake();
        }
    }

    public class OuttakeSlide {
        private DcMotorEx outtakeSlideL;
        private DcMotorEx outtakeSlideR;

        public OuttakeSlide(){
            outtakeSlideL = hardwareMap.get(DcMotorEx.class,"outtakeSlideL");
            outtakeSlideR = hardwareMap.get(DcMotorEx.class,"outtakeSlideR");
            outtakeSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
            outtakeSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class Score implements Action{
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!init){
                    outtakeSlideL.setPower(0.8);
                    outtakeSlideR.setPower(0.8);
                    init = true;
                }
                if (outtakeSlideL.getCurrentPosition() >= armControl.outakeExtendCalc(1.0)){
                    outtakeSlideL.setPower(0);
                    outtakeSlideR.setPower(0);
                    return false;
                }
                return true;
            }
        }

        public Action score(){
            return new Score();
        }

        public class Floor implements Action{
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!init){
                    outtakeSlideL.setPower(-0.5);
                    outtakeSlideR.setPower(-0.5);
                    init = true;
                }
                if (outtakeSlideL.getCurrentPosition() <= armControl.outakeExtendCalc(0)){
                    outtakeSlideL.setPower(0);
                    outtakeSlideR.setPower(0);
                    return false;
                }
                return true;
            }
        }

        public Action floor(){
            return new Floor();
        }
    }

    public class OuttakeSpin{
        Servo outtakeSpin;

        public OuttakeSpin(){
            outtakeSpin = hardwareMap.get(Servo.class,"outtakeSpin");
        }

        public class Horizontal implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                outtakeSpin.setPosition(armControl.outtakeSpinCalc(1));
                return false;
            }
        }

        public Action horizontal(){
            return new Horizontal();
        }
    }
}

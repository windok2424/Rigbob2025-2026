package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLFieldMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;
import com.acmerobotics.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

class Pose2d {
    double x, y, theta;
    Pose2d(double x, double y, double theta) { this.x = x; this.y = y; this.theta = theta; }
}
@TeleOp(name = "aimbot_blue")
public class aimbot_blue extends LinearOpMode {

    double x = 0.0, y = 0.0, theta = 0.0;


    //--Drive Motors--
    DcMotorEx FL;
    DcMotorEx BL;
    DcMotorEx FR;
    DcMotorEx BR;
    //--uhh spin thingies--
    DcMotorEx spin1;
    DcMotorEx spin2;
    Servo blocker;
    IMU imu;

    DcMotorEx turret; //turntable motor
    //DcMotorEx spin; //flywheel

    //Servo upDown; //up and down servo

    final double TICKS_TO_CM = (double) 46 / 9002;
    double lastXWheel = 0.0, lastYWheel = 0.0;

    //--Intake--
    DcMotorEx intake;


    private PIDController turret_pidcontroller;
    private PIDController shooter;
    public static double pshoot = 0.02, ishoot = 0.2, dshoot = 0;
    public static double fshoot = 0;


    public static double pturret = 0.05, iturret = 0, dturret = 0.0005;
    public static double fturret = 0;
    public static double shottarget = -2300;
    public static double turrettarget = 400;


    public void runOpMode() throws InterruptedException {

        waitForStart();

        Limelight3A limelight;

        shooter = new PIDController(pshoot, ishoot, dshoot);
        turret_pidcontroller = new PIDController(pturret, iturret, dturret);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();


        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        spin1 = hardwareMap.get(DcMotorEx.class, "shootup");
        spin2 = hardwareMap.get(DcMotorEx.class, "shootdown");
        blocker = hardwareMap.get(Servo.class, "hold");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

        //elevation = hardwareMap.get(Servo.class, "elevate");


        ElapsedTime runtime = new ElapsedTime();
        telemetry.addData("Current Runtime: ", runtime);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setDirection((DcMotorSimple.Direction.REVERSE));
        //motorBL.setDirection((DcMotorSimple.Direction.REVERSE));
        BR.setDirection((DcMotorSimple.Direction.REVERSE));


        boolean gpp = false;
        boolean pgp = false;
        boolean ppg = false;


        while (opModeIsActive()) {

            //shooter pid

            shooter.setPID(pshoot, ishoot, dshoot);
            turret_pidcontroller.setPID(pturret, iturret, dturret);


            //x=9002ticks =46cm
            //945.2


            double omega = spin1.getVelocity();
            telemetry.addData("this is the omega", omega);


            double pidshot = shooter.calculate(omega, shottarget);
            Pose2d pose = getRobotPose();
            double distance = Math.sqrt(Math.pow(365.76 / 2 - (pose.y), 2) + Math.pow(365.76 + 20.066 + (pose.x), 2));
            telemetry.addData("Distance:", distance);
            telemetry.addData("x distance", pose.x);
            telemetry.addData("y distance", pose.y);
            omega = omega;

            double ff = Math.cos(Math.toRadians(shottarget)) * 0;

            double powershot = pidshot + ff;

            telemetry.addData("thisis pidshot", pidshot);


            telemetry.update();


            //gamepad 1
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;
            //double insanity = gamepad1.right_trigger;

            boolean yes = false;


            boolean slowMode = gamepad1.left_bumper;
            boolean manual_aim = gamepad2.right_bumper;
            double manual_power = gamepad2.right_stick_x;

            boolean inOn = gamepad1.a;
            boolean kick = gamepad2.left_bumper;
            boolean spinny = gamepad2.x;
            boolean close = gamepad2.dpad_down;
            boolean far = gamepad2.dpad_up;
            boolean holdit = gamepad1.b;
            boolean runandkickup = gamepad2.y;
            boolean slowmode = gamepad2.right_bumper;

            double powerFL = (-y - x + rx);
            double powerBL = (y - x - rx);
            double powerFR = (y - x + rx);
            double powerBR = (-y - x - rx);

            if(holdit){
                blocker.setPosition(0);
                intake.setPower(1);

            }
            else{

                blocker.setPosition(1);
            }

            if (far) {
                shottarget = -2200;
            }
            if (close) {
                shottarget = -1900;

            }

            if (inOn) {
                intake.setPower(1);


            } else if(!holdit) {
                intake.setPower(0);


            }
            telemetry.addData("Restarted", powerBL);
            telemetry.update();

            if (slowmode) {
                FL.setPower(powerFL * 0.3);
                BL.setPower(powerBL * 0.3);
                FR.setPower(powerFR * 0.3);
                BR.setPower(powerBR * 0.3);


            } else {
                FL.setPower(powerFL);
                BL.setPower(powerBL);
                FR.setPower(powerFR);
                BR.setPower(powerBR);


            }

            spin1.setPower(1);
            spin2.setPower(1);
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {


                List<FiducialResult> fiducials = result.getFiducialResults();

                for (FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    yes = id == 20;
                    telemetry.addData("Yes", yes);
                    telemetry.update();
                }

                //int id = fiducial.getId(); // The ID number of the fiducial... I hope. nay. I pray.
                double tx = 0;
                double ty = 0;
                double ta = 0;

                telemetry.addData("Pipeline: ", result.getPipelineIndex());
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);

                telemetry.addData("Target Area", ta);

                telemetry.update();

                //THUS IS WHERE TOD ELETE
                //yes = false;

                if(yes) {
                    tx = result.getTx(); // How far left or right the target is (degrees)
                    ty = result.getTy(); // How far up or down the target is (degrees)
                    ta = result.getTa(); // How big the target looks (0%-100% of the image)


                    double turretomega = tx;
                    turrettarget = 0;
                    double turretpower = turret_pidcontroller.calculate(turretomega, turrettarget);
                    boolean manualstop = gamepad1.right_bumper;


                    turret.setPower(turretpower);


                } else{
                    tx = 0;
                    double turretpower = turret_pidcontroller.calculate(tx, turrettarget);
                    turret.setPower(turretpower);
                }

                //int id = fiducial.getFiducialId(); // The ID    c cx cx of the fiducial

                //telemetry.addData("Fiducial: ", id);

                if (manual_aim) {
//        turret.setPower(-manual_power);
//        telemetry.addData("Limelight", "Manual Aim");
//        telemetry.update();
//        } else {
//            turret.setPower(0);
//        }
                }


            }else {
                turret.setPower(0);
                telemetry.addData("Limelight", "No Targets Found");
                telemetry.update();
            }
        }
    }

    Pose2d getRobotPose () {
        double xWheel = BL.getCurrentPosition() * TICKS_TO_CM;
        double yWheel = -FL.getCurrentPosition() * TICKS_TO_CM;

        double deltaXWheel = xWheel - lastXWheel;
        double deltaYWheel = yWheel - lastYWheel;

        lastXWheel = xWheel;
        lastYWheel = yWheel;

        theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double deltaX = deltaXWheel * Math.cos(theta) - deltaYWheel * Math.sin(theta);
        double deltaY = deltaXWheel * Math.sin(theta) + deltaYWheel * Math.cos(theta);

        x += deltaX;
        y += deltaY;

        return new Pose2d(x, y, theta);
    }
}

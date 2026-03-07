package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

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
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "better_blue")
public class McCelary extends LinearOpMode {

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
    Servo adjust;
    IMU imu;
    CRServo transferservo;

    GoBildaPinpointDriver pinpoint;



    DcMotorEx turret; //turntable motor
    //DcMotorEx spin; //flywheel

    //Servo upDown; //up and down servo

    final double TICKS_TO_CM = (double) 46 / 9002;
    double lastXWheel = 0.0, lastYWheel = 0.0;

    //--Intake--
    DcMotorEx intake;


    private PIDController turret_pidcontroller;
    private PIDController shooter;
    public static double pshoot = 0.016, ishoot = 0.04, dshoot = 0;
    public static double fshoot = 0;


    public static double pturret = 0.05, iturret = 0, dturret = 0.0005;
    public static double fturret = 0;
    public static double shottarget = -60;
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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        spin1 = hardwareMap.get(DcMotorEx.class, "shootup");
        spin2 = hardwareMap.get(DcMotorEx.class, "shootdown");
        blocker = hardwareMap.get(Servo.class, "hold");
        adjust = hardwareMap.get(Servo.class, "adjust");
        transferservo = hardwareMap.get(CRServo.class, "transferservo");

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

        FR.setDirection((DcMotorSimple.Direction.REVERSE));
        BR.setDirection((DcMotorSimple.Direction.REVERSE));


        boolean gpp = false;
        boolean pgp = false;
        boolean ppg = false;


        while (opModeIsActive()) {

            //shooter pid

            shooter.setPID(pshoot, ishoot, dshoot);
            turret_pidcontroller.setPID(pturret, iturret, dturret);

            double omega = spin1.getVelocity();
            telemetry.addData("this is the omega", omega);
            //pinpoint.getPosition();
            pinpoint.update();
            omega = omega;


            //gamepad 1
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.left_trigger - gamepad1.right_trigger;
            boolean inOn = gamepad1.a;

            boolean spinnyrev = gamepad1.dpad_left;
            boolean holdit = gamepad1.b;



            boolean adclose = gamepad1.dpad_up;
            boolean adfar = gamepad1.dpad_down;

            //gamepad 2
            boolean check = gamepad2.a;
            boolean manual_aim = gamepad2.left_bumper;
            double manual_power = gamepad2.right_stick_x;
            boolean close = gamepad2.dpad_down;
            boolean far = gamepad2.dpad_up;
            boolean slowmode = gamepad2.right_bumper;

            boolean nolol = gamepad2.b;


            double powerFL = (y - x + rx);
            double powerBL = (y + x + rx);
            double powerFR = (y + x - rx);
            double powerBR = (y - x - rx);

            telemetry.addData("PowerFL", powerFL);
            telemetry.addData("PowerBL", powerBL);
            telemetry.addData("PowerFR", powerFR);
            telemetry.addData("PowerBR", powerBR);

            double distance = 365.76-Math.sqrt(Math.pow(pinpoint.getPosY(DistanceUnit.CM), 2)+Math.pow(pinpoint.getPosX(DistanceUnit.CM)*1.262, 2));
            telemetry.addData("Distance:", distance);
            telemetry.addData("x distance", pinpoint.getPosX(DistanceUnit.CM));
            telemetry.addData("y distance", pinpoint.getPosY(DistanceUnit.CM));
            telemetry.addData("Distance in CM", distance);


            if (far) {
                shottarget = -1800;
                    adjust.setPosition(0.8);

                telemetry.addData("Adfar Works", adfar);
                telemetry.addData("Far Works", shottarget);
            }
            else if (close) {
                telemetry.addData("Close Works", shottarget);



                    adjust.setPosition(0.5);

                telemetry.addData("Adclose Works", adclose);
                shottarget = -1500;

            } else if(nolol){
                shottarget = -60;
                telemetry.addData("No lol", shottarget);
            }else {
                //shottarget = (-2393 + 42.5 * distance + -0.146 * Math.pow(distance, 2) + (1.79 * Math.pow(10, -4)) * Math.pow(distance, 3) + (-1.73 * Math.pow(10, -8)) * Math.pow(distance, 4));
                adjust.setPosition(0.8);
            }
            telemetry.addData("this is shot target", shottarget);

            double pidshot = shooter.calculate(omega, shottarget);

            double ff = Math.cos(Math.toRadians(shottarget)) * 0;

            double powershot = pidshot + ff;



            if(holdit){
                blocker.setPosition(0);
                intake.setPower(-1);
                transferservo.setPower(1);
//                transferservo.setPower(1);
            }
            else{
                blocker.setPosition(0.5);
                transferservo.setPower(0);
            }
            if (inOn) {
                intake.setPower(-1);
                transferservo.setPower(1);


            } else if(spinnyrev){
                intake.setPower(1);
//                transferservo.setPower(-0.5);
            } else if(!holdit) {
                intake.setPower(0);


            }
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
            telemetry.addData("This is powershot", powershot);
            spin1.setPower(powershot);
            spin2.setPower(-powershot);
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && !manual_aim) {
                double tx;
                tx = result.getTx(); // How far left or right the target is (degrees)
                telemetry.addData("Pipeline: ", result.getPipelineIndex());
                telemetry.addData("Target X", tx);
                double turretomega = tx;
                turrettarget = 0;
                double turretpower = turret_pidcontroller.calculate(turretomega, turrettarget);
                boolean manualstop = gamepad1.right_bumper;
                turret.setPower(-turretpower);

            }else if (manual_aim) {
                turret.setPower(manual_power);
                telemetry.addData("Limelight", "Manual Aim");
            }else {
                turret.setPower(0);
                telemetry.addData("Limelight", "No Targets Found");
            }
            telemetry.update();
        }
    }


}

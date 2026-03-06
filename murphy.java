package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLFieldMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
@Autonomous(name="mateo")
public class mato extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;
    public PIDController xpos;
    public PIDController ypos;
    public PIDController thetapos;
    public PIDController shooter;
    //-0.03
    public static double px = -0.03, ix = 0.01, dx = 0.001;
    //0.03

    public static double targetx = 0;
    public static double py = -0.03, iy = 0.01, dy = 0.001;
    //0.03
    public static double targety = 0;
    public static double ptheta = 0.01, itheta = 0, dtheta = 0.0001;
    public static double targettheta = 0;
    public PIDController rotation;
    public static double prot = 0.08, irot = 0, drot = 0.00001;
    public static double targetrot = 0;
    public static double pturret = 0.05, iturret = 0, dturret = 0.0005;
    public static double pshoot = 0.2, ishoot = 0.1, dshoot = 0;
    public static double f = 0;
    private PIDController turret_pidcontroller;
    public static double turrettarget = 400;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotorEx spin1;
    DcMotorEx spin2;
    //--Intake--
    DcMotor intake;
    //    Servo holdfast;
    IMU imu;
    DcMotor turret;
    CRServo transferservo;
    Limelight3A limelight;
    Servo blocker;

    double x = 0.0, y = 0.0, theta = 0.0;
    double lastXWheel = 0.0, lastYWheel = 0.0;

    final double TICKS_PER_REV = 8192;
    final double WHEEL_DIAMETER_CM = 3.0;
    final double TICKS_TO_CM = (double) 46 / 9002;

    //final double Kp_xy = 0.14;
    final double Kp_xy = 0.3;
    final double Kp_theta = 0.12;
    //    public PIDController shooter;
//    public static double pshoot = 0.02, ishoot = 0.2, dshoot = 0;
    private double shottarget = -2340;

    final double MIN_TRANSLATION_POWER = 0.1;
    final double MIN_ROTATION_POWER = 0.05;

    final double POSITION_TOLERANCE_CM = 5.0;
    final double ANGLE_TOLERANCE_RAD = Math.toRadians(2);
    class Pose2d {
        double x, y, theta;
        Pose2d(double x, double y, double theta) { this.x = x; this.y = y; this.theta = theta; }
    }
    Pose2d getRobotPose () {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.update();
        double xWheel = -pinpoint.getPosX(DistanceUnit.CM);
        double yWheel = pinpoint.getPosY(DistanceUnit.CM);

        double deltaXWheel = xWheel - lastXWheel;
        double deltaYWheel = yWheel - lastYWheel;

        lastXWheel = xWheel;
        lastYWheel = yWheel;

//       theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        theta = pinpoint.getHeading(AngleUnit.DEGREES);

        double deltaX = deltaXWheel * Math.cos(theta) - deltaYWheel * Math.sin(theta);
        double deltaY = deltaXWheel * Math.sin(theta) + deltaYWheel * Math.cos(theta);

        x += deltaX;
        y += deltaY;

        return new Pose2d(x, y,theta);
    }

    @Override
    public void runOpMode() {
        xpos = new PIDController(px, ix , dx);
        ypos = new PIDController(py, iy , dy);
        thetapos = new PIDController(ptheta, itheta , dtheta);
        rotation = new PIDController(prot, irot, drot);
        shooter = new PIDController(pshoot,ishoot,dshoot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        shooter = new PIDController(pshoot, ishoot, dshoot);
//        turret_pidcontroller = new PIDController(pturret, iturret, dturret);


        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        transferservo = hardwareMap.get(CRServo.class, "transferservo");
        blocker = hardwareMap.get(Servo.class, "hold");
//        intake = hardwareMap.get(DcMotor.class, "intake");
        spin1 = hardwareMap.get(DcMotorEx.class, "shootup");
        spin2 = hardwareMap.get(DcMotorEx.class, "shootdown");
//        turret = hardwareMap.get(DcMotor.class, "turret");
//        holdfast = hardwareMap.get(Servo.class, "hold");
//
//        turret_pidcontroller.setPID(pturret, iturret, dturret);
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        pinpoint.resetPosAndIMU();
        waitForStart();


        if (opModeIsActive()) {

            pinpoint.update();
            Pose2d pose = getRobotPose();

            telemetry.addData("X (mm)", getRobotPose().x);
            telemetry.addData("Y (mm)", getRobotPose().y);
            telemetry.addData("Heading (rad)", getRobotPose().theta);
            telemetry.update();
            imu.resetYaw();
            sleep(250);


            move( -100,-100,0,2, false);
            shoot(2,1400,20);
            move(-10,-100,0,2, false);
            move(-10,-120,0,0.5, false);
            move(10,-120,0,2, false);
            move(-100,-100,0,3, false);
            shoot(2,1400,20);
            move(-50,-150,0,2, false);
            move(-10,-150,0,1, false);
            move(-100,-100,0,2, false);
            shoot(2,1400,20);
            move(-50,-200,0,2, false);
            move(-10,-200,0,1, false);
            move(-100,-100,0,2, false);
            shoot(2,1400,20);
            move(-50,-250,0,2, false);
            move(-10,-250,0,1, false);
            move(-100,-100,0,4, false);
            shoot(2,1400,20);
        }

        //setMecanumPowers(0,0,0);
    }


    double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    void rotate(double angle) {
        while (opModeIsActive()) {
            targetrot = angle;
            rotation.setPID(prot, irot, drot);
            double omegarot = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("omega :", omegarot);
            telemetry.addData("target: ", targetrot);
            telemetry.update();

            double pidshot = rotation.calculate(omegarot, targetrot);

            double ff = Math.cos(Math.toRadians(targetrot)) * 0;

            double pow = pidshot + ff;
            double powerFL = +pow;
            double powerBL = -pow;
            double powerFR = +pow;
            double powerBR = -pow;
            frontLeft.setPower(powerFL);
            backLeft.setPower(powerBL);
            frontRight.setPower(powerFR);
            backRight.setPower(powerBR);
            double errorofit = targetrot - omegarot;


            if (-0.3 < errorofit && errorofit < 0.3) {
                break;

            }

        }

    }

    void justrpm(double last) {
        ElapsedTime sec = new ElapsedTime();


        sec.reset();
        double current = sec.seconds();
        while (opModeIsActive()) {
            current = sec.seconds();
        }


    }
    void move(double y, double x, double angle, double last, boolean runintake) {
        ElapsedTime sec = new ElapsedTime();


        sec.reset();
        intake = hardwareMap.get(DcMotor.class, "intake");

        double current = sec.seconds();
        while (opModeIsActive()) {
            if(runintake = true){
                blocker.setPosition(0.8);
                intake.setPower(-1);
                transferservo.setPower(1);
            }else{
                intake.setPower(0);
            }

            current = sec.seconds();


            pinpoint.update();
            targetx = x;
            targety = y;
            targettheta = angle;
            xpos.setPID(px, ix, dx);
            ypos.setPID(py, iy, dx);
            thetapos.setPID(ptheta, itheta, dtheta);
            double currentx = -pinpoint.getPosX(DistanceUnit.CM);
            double currenty = -pinpoint.getPosY(DistanceUnit.CM);
            double currenttheta = pinpoint.getHeading(AngleUnit.DEGREES);
            double errorX = targetx - currentx;
            double errorY = targety - currenty;
            double errortheta = targettheta - currenttheta;
            double pidx = xpos.calculate(currentx, targetx);
            double pidy = ypos.calculate(currenty, targety);
            double pidtheta = thetapos.calculate(currenttheta, targettheta);
            telemetry.addData("Target", "(%.1f, %.1f, %.1f)", targetx, targety, targettheta);
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", currentx, currenty, currenttheta);
            telemetry.addData("PID", "(%.1f, %.1f, %.1f°)", pidx, pidy, pidtheta);
            telemetry.update();
            setMecanumPowers(pidx, pidy, pidtheta);
            if (Math.abs(errorX) < POSITION_TOLERANCE_CM &&
                    Math.abs(errorY) < POSITION_TOLERANCE_CM &&
                    Math.abs(errortheta) < ANGLE_TOLERANCE_RAD) {
                break;
            }

            if(current >= last){

                break;
            }


        }

    }

    void setMecanumPowers ( double strafe, double forward, double rotate){
        double fl = -forward - strafe - rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
    void holdfastball ( double last){
        ElapsedTime sec = new ElapsedTime();


        sec.reset();
        double current = sec.seconds();
//            while (opModeIsActive()) {
//                current = sec.seconds();
//                holdfast.setPosition(1);
//                if (current == last) {
//                    holdfast.setPosition(0);
//                    break;
//
//                }
//            }


    }
    void shoot(double last, double rpm, double target){
        ElapsedTime sec = new ElapsedTime();



        sec.reset();
        double current = sec.seconds();
        shottarget = rpm;


        ElapsedTime clockfortpr = new ElapsedTime();




        sec.reset();
        double tprinsec = clockfortpr.seconds();

        while(opModeIsActive()){
            blocker.setPosition(0);
            intake.setPower(-1);
            transferservo.setPower(1);





            current = sec.seconds();
            tprinsec = clockfortpr.seconds();

            if(0< tprinsec && tprinsec < 4){
                intake.setPower(0);

            }

            else{
                intake.setPower(0);
            }




            //shooter pid

            shooter.setPID(pshoot, ishoot, dshoot);



            double omega = spin1.getVelocity();
            telemetry.addData("this is the omega", omega);


            double pidshot = shooter.calculate(omega, shottarget);
            omega = omega;


            double powershot = pidshot;

            telemetry.addData("thisis pidshot", pidshot);


            telemetry.update();

            spin1.setPower(-powershot);
            spin2.setPower(powershot);

            if(current >  last){
                break;

            }
        }



    }
}

package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLFieldMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.arcrobotics.ftclib.controller.PIDController;

import java.util.List;


@Autonomous(name = "stupid")
public class bullsheisse extends LinearOpMode {

    DcMotor intake;
    Servo blocker;
    Limelight3A limelight;
    DcMotor spin1;
    DcMotor spin2;

    DcMotor turret;

    private PIDController turret_pidcontroller;
    private PIDController shooter;
    public static double pshoot = 0.02, ishoot = 0.2, dshoot = 0;
    public static double fshoot = 0;


    public static double pturret = 0.05, iturret = 0, dturret = 0.0005;
    public static double fturret = 0;
    public static double shottarget = -2300;
    public static double turrettarget = 400;


    public void runOpMode() {



    }
    void shooter(double last, boolean areyousure){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

        PIDController shooter = new PIDController(pshoot, ishoot, dshoot);
        PIDController turret_pidcontroller = new PIDController(pturret, iturret, dturret);
        shooter.setPID(pshoot, ishoot, dshoot);
        turret_pidcontroller.setPID(pturret, iturret, dturret);

        boolean yes = false;
        ElapsedTime seconds = new ElapsedTime();
        seconds.reset();

        while(opModeIsActive()) {


            double omega = spin1.getVelocity();
            telemetry.addData("this is the omega", omega);


            double pidshot = shooter.calculate(omega, shottarget);
            omega = omega;

            double ff = Math.cos(Math.toRadians(shottarget)) * 0;

            double powershot = pidshot + ff;
            double secs = seconds.seconds();

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

                if (yes) {
                    tx = result.getTx(); // How far left or right the target is (degrees)
                    ty = result.getTy(); // How far up or down the target is (degrees)
                    ta = result.getTa(); // How big the target looks (0%-100% of the image)


                    double turretomega = tx;
                    turrettarget = 0;
                    double turretpower = turret_pidcontroller.calculate(turretomega, turrettarget);
                    boolean manualstop = gamepad1.right_bumper;


                    turret.setPower(turretpower);


                } else {
                    tx = 0;
                    double turretpower = turret_pidcontroller.calculate(tx, turrettarget);
                    turret.setPower(turretpower);
                }

                //int id = fiducial.getFiducialId(); // The ID    c cx cx of the fiducial

                //telemetry.addData("Fiducial: ", id);


            } else {
                turret.setPower(0);
                telemetry.addData("Limelight", "No Targets Found");
                telemetry.update();
            }


            if (areyousure) {
                spin1.setPower(1);
                spin2.setPower(1);
                blocker.setPosition(1);
                intake.setPower(1);
            } else {
                spin1.setPower(0);
                spin2.setPower(0);
                blocker.setPosition(0);
                intake.setPower(0);
            }
            if (secs > last) {
                break;
            }
        }
    }
    void intake(boolean doit, double last){
        if (doit){
            intake.setPower(1);
            blocker.setPosition(0);
        } else{
            intake.setPower(0);
        }
    }

}

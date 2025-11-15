package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLFieldMap;

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

import java.util.List;

@TeleOp(name = "aimbot_blue")
public class aimbot_blue extends LinearOpMode {

    //--Drive Motors--
    DcMotorEx FL;
    DcMotorEx BL;
    DcMotorEx FR;
    DcMotorEx BR;
    //--uhh spin thingies--
    DcMotorEx spin1;
    DcMotorEx spin2;
    //--Turret Stuff--
    Servo elevation;

    DcMotorEx turret; //turntable motor
    //DcMotorEx spin; //flywheel

    //Servo upDown; //up and down servo
    Limelight3A limelight;

    //--Intake--
    DcMotorEx intake;
    CRServo roll_left;
    CRServo roll_right;

    CRServo up_left;
    CRServo up_right;

    Servo kickup;

    //@Override
    //Limelight3A limelight;



    public void runOpMode() throws InterruptedException{

        waitForStart();


        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        spin1 = hardwareMap.get(DcMotorEx.class, "shootup");
        spin2 = hardwareMap.get(DcMotorEx.class, "shootdown");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        roll_left = hardwareMap.get(CRServo.class, "inspin1");
        roll_right = hardwareMap.get(CRServo.class, "inspin2");

        up_left = hardwareMap.get(CRServo.class, "spinup1");
        up_right = hardwareMap.get(CRServo.class, "spinup2");

        kickup = hardwareMap.get(Servo.class, "kickup");

        //elevation = hardwareMap.get(Servo.class, "elevate");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

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


        while(opModeIsActive()) {
            //gamepad 1
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            boolean slowMode = gamepad1.left_stick_button;
            boolean manual_aim = gamepad2.left_bumper;
            double manual_power = gamepad2.right_stick_x;

            boolean inOn = gamepad1.a;
            boolean kick = gamepad1.b;
            boolean spinny = gamepad1.x;

            double insanity = gamepad1.right_trigger;

            double powerFL = (-y - x + rx);
            double powerBL = (y - x - rx);
            double powerFR = (y - x + rx);
            double powerBR = (-y - x - rx);


            spin1.setPower(-insanity);
            spin2.setPower(-insanity);

            if(inOn){
                intake.setPower(1);


            }else{
                intake.setPower(0);


            }
            if(spinny){
                roll_left.setPower(1);
                roll_right.setPower(-1);
                up_left.setPower(-1);
                up_right.setPower(1);
            } else{
                roll_left.setPower(0);
                roll_right.setPower(0);
                up_left.setPower(0);
                up_right.setPower(0);
            }
            if(kick){
                kickup.setPosition(0);
            }else{
                kickup.setPosition(1);
            }


//    if (gamepad1.a && toggleReady) {
//        toggleReady = false; // Set toggle to not ready to prevent re-triggering
//
//        // Toggle the servo state
//        servoState = !servoState;
//
//        // Set servo position based on the new state
//        if (servoState) {
//            .setPosition(1.0); // Example: 1.0 for open position
//        } else {
//            myServo.setPosition(0.0); // Example: 0.0 for closed position
//        }
//    }

            // Reset toggleReady when the button is released
//    if (!gamepad1.a) {
//        toggleReady = true;
//    }


            //telemetry.addData("Restarted", powerBL);
            telemetry.update();
            FL.setPower(powerFL);
            BL.setPower(powerBL);
            FR.setPower(powerFR);
            BR.setPower(powerBR);
            //turret.setPower(1); bad idea to uncomment this. Run it only for 2 seconds
            LLResult result = limelight.getLatestResult();
            boolean yes = false;
            if (result != null && result.isValid() && !manual_aim) {


                List<FiducialResult> fiducials = result.getFiducialResults();

                for (FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    if (id == 20) {
                        yes = true;
                    } else if(id == 21){
                        gpp = true;
                    } else if(id == 22){
                        pgp = true;
                    } else if(id == 23){
                        ppg = true;
                    }
                    telemetry.addData("finished", powerBL);
                }

                //int id = fiducial.getId(); // The ID number of the fiducial... I hope. nay. I pray.

                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Pipeline: ", result.getPipelineIndex());
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Distance", Distance(ta));
                telemetry.addData("Target Area", ta);

                telemetry.update();
                //int id = fiducial.getFiducialId(); // The ID    c cx cx of the fiducial

                //telemetry.addData("Fiducial: ", id);

                if(tx > 3 && yes) {
                    turret.setPower(-0.8);
                    sleep(10);
                } else if(tx > 1 && yes){
                    turret.setPower(-0.2);
                    sleep(10);
                } else if(tx < -1 && yes){
                    turret.setPower(0.2);
                    sleep(10);
                } else if(tx < -3 && yes) {
                    turret.setPower(0.8);
                    sleep(10);
                } else {
                    turret.setPower(0);

                }
            } else if(manual_aim) {
                turret.setPower(-manual_power);
                telemetry.addData("Limelight", "Manual Aim");
                telemetry.update();
            } else{
                turret.setPower(0);
                telemetry.addData("Limelight", "No Targets Found");
                telemetry.update();
            }


        }



    }

    public double Distance(double ta){
        //what is this dumbahh thing
        double scale = 30665.95; //placeholder value, subject to change.
        double dist = (scale / ((ta*100)*(ta*100)));
        return dist;

    }
}

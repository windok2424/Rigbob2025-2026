package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLFieldMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "aimbot_blue")
public class aimbot_blue extends LinearOpMode {
    DcMotorEx FL;
    DcMotorEx BL;
    DcMotorEx FR;
    DcMotorEx BR;

    DcMotorEx turret; //turntable motor
    //DcMotorEx spin; //flywheel

    //Servo upDown; //up and down servo
    Limelight3A limelight;

    //@Override
    //Limelight3A limelight;



    public void runOpMode() throws InterruptedException{

        waitForStart();


        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

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

        FL.setDirection((DcMotorSimple.Direction.REVERSE));
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

    double powerFL = (y + x + rx);
    double powerBL = (y - x + rx);
    double powerFR = (y - x - rx);
    double powerBR = (y + x - rx);

    
    telemetry.addData("Restarted", powerBL);
    telemetry.update();
    FL.setPower(powerFL);
    BL.setPower(powerBL);
    FR.setPower(powerFR);
    BR.setPower(powerBR);
    //turret.setPower(1); bad idea to uncomment this. Run it only for 2 seconds
    LLResult result = limelight.getLatestResult();
    boolean yes = false;
    if (result != null && result.isValid()) {


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
        telemetry.addData("Target Area", ta);

telemetry.update();
            //int id = fiducial.getFiducialId(); // The ID    c cx cx of the fiducial

            //telemetry.addData("Fiducial: ", id);

            if (tx > 7 && yes) {
                turret.setPower(0.4);
                sleep(10);
                continue;
            } else if (tx < -7 && yes) {
                turret.setPower(-0.4);
                sleep(10);
                continue;
            } else {
                turret.setPower(0);
                continue;
            }
        } else {
        turret.setPower(0);
        telemetry.addData("Limelight", "No Targets");
        telemetry.update();
    }


}



    }
}

package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="betterbot_blue")
public class betterbot_blue extends LinearOpMode {
    //--Driver bums--
    DcMotorEx FL;
    DcMotorEx BL;
    DcMotorEx FR;
    DcMotorEx BR;

    //--Intake and Holder--
    DcMotorEx intake;
    Servo holder;

    //--Aiming Systems and Fire Control--
    DcMotorEx turret;
    Limelight3A lime;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        lime = hardwareMap.get(Limelight3A.class, "limelight");
        lime.start();

        //0 FOR BLUE, 1 FOR RED
        lime.pipelineSwitch(0);


        turret = hardwareMap.get(DcMotorEx.class, "turret");

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        holder = hardwareMap.get(Servo.class, "hold");

        while(opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.left_trigger - gamepad1.right_trigger;

            boolean hold = gamepad1.b;
            boolean in = gamepad1.a;

            boolean out = gamepad1.dpad_left;

            boolean manual = gamepad2.left_bumper;
            double manualpower = gamepad2.right_stick_x;

            boolean close;
            boolean far;

            double powerFL = (y - x + rx);
            double powerBL = (y + x + rx);
            double powerFR = (y + x - rx);
            double powerBR = (y - x - rx);

            FL.setPower(powerFL);
            FR.setPower(powerFR);
            BL.setPower(powerBL);
            BR.setPower(powerBR);

            if(in){
                //change as needed
                holder.setPosition(0);
                intake.setPower(1);
            } else if(hold){
                holder.setPosition(1);
                intake.setPower(0.8);
            } else if(out){
                intake.setPower(-1);
            }else {
                holder.setPosition(0);
                intake.setPower(0);
            }

            LLResult result = lime.getLatestResult();
            if(result != null && result.isValid() && !manual){
                double tx;
                tx = result.getTx();
                //ADD PID
                telemetry.addData("Found it:", 2);

            } else if(manual){
                turret.setPower(manualpower);
                telemetry.addData("Manual Aim", 1);
            } else{
                telemetry.addData("No targets", 0);
            }


            telemetry.update();
        }

    }
}

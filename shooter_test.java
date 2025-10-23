package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLFieldMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "shooter Test")
public class shooter_test extends LinearOpMode{
    DcMotorEx shootup;
    DcMotorEx shootdown;

    DcMotorEx FL;
    DcMotorEx BL;
    DcMotorEx FR;
    DcMotorEx BR;

    DcMotorEx turret;

    public void runOpMode() throws InterruptedException{
        waitForStart();

        shootup = hardwareMap.get(DcMotorEx.class, "shootup");
        shootdown = hardwareMap.get(DcMotorEx.class, "shootdown");


        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        turret = hardwareMap.get(DcMotorEx.class, "turret");



        while(opModeIsActive()){

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            boolean slowMode = gamepad1.left_stick_button;

            double powerFL = (-y - x + rx);
            double powerBL = (y - x - rx);
            double powerFR = (y - x + rx);
            double powerBR = (-y - x - rx);

            double spin = gamepad2.right_stick_x;

            FL.setPower(powerFL);
            BL.setPower(powerBL);
            FR.setPower(powerFR);
            BR.setPower(powerBR);

            turret.setPower(spin);


            shootup.setPower(-1);
            shootdown.setPower(-1);
        }

    }

}

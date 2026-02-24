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

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        //intake = hardwareMap.get(DcMotorEx.class, "intake");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        holder = hardwareMap.get(Servo.class, "holder");

        while(opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.left_trigger - gamepad1.right_trigger;

            double powerFL = (y - x + rx);
            double powerBL = (y + x + rx);
            double powerFR = (y + x - rx);
            double powerBR = (y - x - rx);

            FL.setPower(powerFL);
            FR.setPower(powerFR);
            BL.setPower(powerBL);
            BR.setPower(powerBR);
        }

    }
}

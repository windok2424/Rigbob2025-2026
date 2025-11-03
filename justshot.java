package org.firstinspires.ftc.teamcode;
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


@Config
@TeleOp
public class justshot extends OpMode {

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_rev = 537.6 ;
    private DcMotorEx shootup;
    private DcMotorEx shootdown;
    @Override
    public void init(){
        controller = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shootup = hardwareMap.get(DcMotorEx.class, "shootup");
        shootdown = hardwareMap.get(DcMotorEx.class, "shootdown");

    }
    public void loop(){
        controller.setPID(p, i ,d);
        double omega = shootup.getVelocity();

        double pid = controller.calculate(omega, target);
        omega = omega;
        double ff = Math.cos(Math.toRadians(target)) * f;

        double power = pid + ff;

        shootup.setPower(power);
        shootdown.setPower(power);

        telemetry.addData("omega :", omega);
        telemetry.addData("target: ", target);
        telemetry.update();
    }

}
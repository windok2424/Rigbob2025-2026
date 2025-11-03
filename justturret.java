package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class justturret extends OpMode {

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_rev = 2150;
    private final double rev_in_degree = 360;
    private DcMotorEx turret;

    double omega = 0;
    @Override
    public void init(){
        omega = 0;
        controller = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = hardwareMap.get(DcMotorEx.class, "turret");

    }
    public void loop(){
        controller.setPID(p, i ,d);
        omega = turret.getCurrentPosition();
        //620 ticks is 90 deg
        omega = omega;
        double pid = controller.calculate(omega, target);

        double ff = Math.cos(Math.toRadians(target)) * f;

        double power = pid + ff;

        turret.setPower(-power);


        telemetry.addData("omega :", omega);
        telemetry.addData("target: ", target);
        telemetry.update();
    }

}
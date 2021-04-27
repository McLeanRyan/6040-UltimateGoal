package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Thread.sleep;

@TeleOp
public class RobodogsTeleOp extends OpMode {

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotorEx flywheel = null;
    private DcMotor intake = null;
    private Servo cache = null;
    private Servo push = null;
    double cacheStartPos;

    @Override
    public void init() {
    leftFront = hardwareMap.dcMotor.get("leftFront");
    leftBack = hardwareMap.dcMotor.get("leftBack");
    rightFront = hardwareMap.dcMotor.get("rightFront");
    rightBack = hardwareMap.dcMotor.get("rightBack");
    flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    intake = hardwareMap.dcMotor.get("intake");
    cache = hardwareMap.servo.get("cache");
    push = hardwareMap.servo.get("push");

    //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    cacheStartPos = cache.getPosition();

    telemetry.addData("Cache Pos:", cache.getPosition());
    telemetry.addLine("Initialization Complete");
    telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addData("Cache Pos:", cache.getPosition());
        telemetry.addData("leftFront Power:", leftFront.getPower());
        telemetry.addData("leftBack Power:", leftBack.getPower());
        telemetry.addData("rightFront Power:", rightFront.getPower());
        telemetry.addData("rightBack Power:", rightBack.getPower());
        telemetry.addData("flywheel Power:", flywheel.getPower());
        telemetry.addData("intake Power:", intake.getPower());
        telemetry.addData("flywheel Velocity: ", flywheel.getVelocity(AngleUnit.RADIANS)+" Radians per Second");
        telemetry.update();

        double px = gamepad1.left_stick_x;
        if (Math.abs(px) < 0.05) px = 0;
        double py = -gamepad1.left_stick_y;
        if (Math.abs(py) < 0.05) py = 0;
        double pa = -(gamepad1.right_stick_x*(.70));
        if (Math.abs(pa) < 0.05) pa = 0;
        double plf = -px + py - pa;
        double plb = px + py + -pa;
        double prf = -px + py + pa;
        double prb = px + py + pa;
        double max = Math.max(1.0, Math.abs(plf));
        max = Math.max(max, Math.abs(plb));
        max = Math.max(max, Math.abs(prf));
        max = Math.max(max, Math.abs(prb));
        plf /= max;
        plb /= max;
        prf /= max;
        prb /= max;
        leftFront.setPower(plf);
        leftBack.setPower(plb);
        rightFront.setPower(prf);
        rightBack.setPower(prb);

        while (gamepad1.right_trigger > 0)    {
            leftFront.setPower(1);
            leftBack.setPower(-1);
            rightFront.setPower(-1);
            rightBack.setPower(1);
        }

        while (gamepad1.left_trigger > 0)       {
            leftFront.setPower(-1);
            leftBack.setPower(1);
            rightFront.setPower(1);
            rightBack.setPower(-1);
        }

        while (gamepad1.right_bumper)    {
            leftFront.setPower(0.5);
            leftBack.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightBack.setPower(0.5);
        }

        while (gamepad1.left_bumper)       {
            leftFront.setPower(-0.5);
            leftBack.setPower(0.5);
            rightFront.setPower(0.5);
            rightBack.setPower(-0.5);
        }

        if (gamepad2.left_bumper) {
            flywheel.setPower(1);
        }else {
            flywheel.setPower(0);
        }

        if (gamepad2.left_trigger > 0.25) {
            flywheel.setPower(-1);
        }else {
            flywheel.setPower(0);
        }

        if (gamepad2.x) {
            push.setPosition(push.getPosition()+0.5);
        }

        if (gamepad2.y) {
            push.setPosition(push.getPosition()-0.5);
        }

        /*if (gamepad2.left_bumper) {
            flywheel.setVelocity(33.33, AngleUnit.RADIANS);
        }else {
            flywheel.setVelocity(0, AngleUnit.RADIANS);
        }
        if(gamepad2.left_trigger > 0.25) {
            flywheel.setVelocity(-8.33, AngleUnit.RADIANS);
        }else {
            flywheel.setVelocity(0, AngleUnit.RADIANS);
        }*/

        if (gamepad2.right_bumper) {
            intake.setPower(1);
        }else {
            intake.setPower(0);
        }

        if (gamepad2.right_trigger > 0.25) {
            intake.setPower(-0.5);
        } else {
            intake.setPower(0);
        }

        if (gamepad2.a) {
            cache.setPosition(cache.getPosition()+0.25);
       }
        if(gamepad2.b) {
            cache.setPosition(cache.getPosition()-0.25);
        }
    }
}
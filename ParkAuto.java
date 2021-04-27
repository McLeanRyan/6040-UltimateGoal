package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
@Autonomous
public class ParkAuto extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor flywheel = null;
    private DcMotor intake = null;
    private Servo cache = null;

    BNO055IMU imu;
    private Orientation angles;

    double kP = 0.005;

    double totalError = 0;
    double lastAngle = 0;

    double ticksPerMotorRev = 560;
    double driveGearReduction = 1;
    double wheelDiameterInches = 3.77953;
    double ticksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        intake = hardwareMap.dcMotor.get("intake");
        cache = hardwareMap.servo.get("cache");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("leftFront Starting Pos:", leftFront.getCurrentPosition());
        telemetry.addData("leftBack Starting Pos:", leftBack.getCurrentPosition());
        telemetry.addData("rightFront starting Pos:", rightFront.getCurrentPosition());
        telemetry.addData("rightBack starting Pos:", rightBack.getCurrentPosition());
        telemetry.update();

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();       //sets up IMU
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "ControlHubIMUCalibration.json";
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        waitForStart();

        encoderDrive(0.5, 60, false);

        stop();

    }

    private void encoderDrive(double speed, double inches, boolean strafe) {

        telemetry.addLine("Encoder Drive");

        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
        int lFPos = leftFront.getCurrentPosition();
        int rFPos = rightFront.getCurrentPosition();
        int lBPos = leftBack.getCurrentPosition();
        int rBPos = rightBack.getCurrentPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;

        if (opModeIsActive()) {
            if (strafe) {
                newLFTarget = lFPos + (int) (inches * ticksPerInch);
                newRFTarget = rFPos - (int) (inches * ticksPerInch);
                newLBTarget = lBPos - (int) (inches * ticksPerInch);
                newRBTarget = rBPos + (int) (inches * ticksPerInch);
            } else {
                newLFTarget = lFPos + (int) (inches * ticksPerInch);
                newRFTarget = rFPos + (int) (inches * ticksPerInch);
                newLBTarget = lBPos + (int) (inches * ticksPerInch);
                newRBTarget = rBPos + (int) (inches * ticksPerInch);
            }

            telemetry.addData("speed", speed);
            telemetry.addData("inches", inches);
            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);

            leftFront.setTargetPosition(newLFTarget);
            rightFront.setTargetPosition(newRFTarget);
            leftBack.setTargetPosition(newLBTarget);
            rightBack.setTargetPosition(newRBTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            while (opModeIsActive() && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                if (!strafe) {
                    double error = kP * (startAngle - angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Start Angle", startAngle);
                    telemetry.addData("Current Angle", angles.firstAngle);
                    telemetry.addData("error", error);
                    telemetry.update();
                    if (error > 0) {
                        leftFront.setPower(Math.abs(speed) - error);
                        rightFront.setPower(Math.abs(speed) + error);
                        leftBack.setPower(Math.abs(speed) - error);
                        rightBack.setPower(Math.abs(speed) + error);
                    } else if (error < 0) {
                        leftFront.setPower(Math.abs(speed) + error);
                        rightFront.setPower(Math.abs(speed) - error);
                        leftBack.setPower(Math.abs(speed) + error);
                        rightBack.setPower(Math.abs(speed) - error);
                    }
                }
                telemetry.addData("LF Current Position", leftFront.getCurrentPosition());
                telemetry.addData("RF Current Position", rightFront.getCurrentPosition());
                telemetry.addData("LB Current Position", leftBack.getCurrentPosition());
                telemetry.addData("RB Current Position", rightBack.getCurrentPosition());
                telemetry.addData("LF Current Power", leftFront.getPower());
                telemetry.addData("RF Current Power", rightFront.getPower());
                telemetry.addData("LB Current Power", leftBack.getPower());
                telemetry.addData("RB Current Power", rightBack.getPower());
                telemetry.update();
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);
        }
    }
}

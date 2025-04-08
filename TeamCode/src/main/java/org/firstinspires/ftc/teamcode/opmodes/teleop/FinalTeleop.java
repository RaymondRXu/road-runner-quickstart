package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="Chesapeake Teleop")
public class FinalTeleop extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    public Servo bridge;
    public Servo sampleClaw;
    public Servo specimenArm;
    public Servo specimenClaw;
    public Servo rail;
    public Servo rotation;
    public Servo ledIndicator;

    private IMU imu;
    public double DRIVE_POWER_VARIABLE = 1;
    public boolean specimenIsOpen;
    public boolean sampleIsOpen;
    public double specimenPosition;
    public double samplePosition;
    public ElapsedTime specimenTimer;
    public ElapsedTime sampleTimer;
    // public ElapsedTime ledTimer;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ledIndicator = hardwareMap.get(Servo.class, "led");
        //initializing the imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        //adding the orientation of the imu for accurate calculations
        imu.initialize(parameters);
        // Reset IMU heading
        imu.resetYaw();

        /* servo initialize*/
        bridge = hardwareMap.get(Servo.class, "bridge");
        sampleClaw = hardwareMap.get(Servo.class, "sampleClaw");
        specimenArm = hardwareMap.get(Servo.class, "specimenArm");
        specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
        rail = hardwareMap.get(Servo.class, "rail");
        rotation = hardwareMap.get(Servo.class, "rotation");

        this.rail.setPosition(0);
        this.specimenClaw.setPosition(0);
        specimenIsOpen = true;
        sampleIsOpen = true;
        specimenPosition = 0.0;
        samplePosition = 0.8;
        specimenTimer = new ElapsedTime();
        sampleTimer = new ElapsedTime();
        //ledTimer = new ElapsedTime();
        //ledTimer.reset();
        //ledIndicator.setPosition(0.5);
    }

    @Override
    public void loop() {

        // else if (gamepad2.dpad_left)

        // Get drive inputs (negated Y because joystick Y is reversed)
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;  // Negated to match standard coordinate system
        double rx = -DRIVE_POWER_VARIABLE * (gamepad1.right_trigger - gamepad1.left_trigger);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Apply field centric transformation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate motor powers using transformed inputs
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        frontLeft.setPower(frontLeftPower * DRIVE_POWER_VARIABLE);
        frontRight.setPower(frontRightPower * DRIVE_POWER_VARIABLE);
        backLeft.setPower(backLeftPower * DRIVE_POWER_VARIABLE);
        backRight.setPower(backRightPower * DRIVE_POWER_VARIABLE);

        // led
        /*if (ledTimer.milliseconds() < 60000)
            ledIndicator.setPosition(0.5);
        else if (ledTimer.milliseconds() < 90000)
            ledIndicator.setPosition(0.388);
        else
            ledIndicator.setPosition(0.279);*/

        // Debug telemetry
        if (rail.getPosition() >= 0.45)
            telemetry.addData("Rail Position", "UP");
        else
            telemetry.addData("Rail Position", "DOWN");
        telemetry.addData("railPosition", rail.getPosition());
        telemetry.addData("Robot Heading", Math.toDegrees(botHeading));
        telemetry.addData("left stick y", y);
        telemetry.addData("left stick x ", x);
        telemetry.addData("armPos", specimenArm.getPosition());
        telemetry.addData("sampleIsOpen", sampleIsOpen);
        telemetry.addData("specimenIsOpen",specimenIsOpen);
        telemetry.update();

        // gamepad1 - movement systems (Malek)
        if (gamepad1.left_bumper) DRIVE_POWER_VARIABLE = 0.5; // slow driving
        else if (gamepad1.right_bumper) DRIVE_POWER_VARIABLE = 1; // revert to normal speed (fast)
        else if (gamepad1.b && sampleTimer.milliseconds() > 250) sampleClaw.setPosition(0); // opens sample claw
        else if (gamepad1.y && sampleTimer.milliseconds() > 250
        ) sampleClaw.setPosition(0.8); // closes sample claw
        else if (gamepad1.x) imu.resetYaw();

        // gamepad2 - intake systems (Kaichen)
        if (gamepad2.left_bumper && sampleTimer.milliseconds() > 250) { // opens/closes sample claw
            if (sampleIsOpen) {
                sampleClaw.setPosition(0.8);
                sampleIsOpen = false;
            } else {
                sampleClaw.setPosition(0.0);
                sampleIsOpen = true;
            }
            sampleTimer.reset();
        } else if (gamepad2.right_bumper && specimenTimer.milliseconds() > 250) { // open/closes specimen claw
            if (specimenIsOpen) {
                specimenClaw.setPosition(1.0);
                specimenIsOpen = false;
            } else {
                specimenClaw.setPosition(0.0);
                specimenIsOpen = true;
            }
            specimenTimer.reset();
        } else if (gamepad2.dpad_up) { // specimen arm goes up
            this.rail.setPosition(0.61); //0.49
            this.specimenArm.setPosition(0.83);
        } else if (gamepad2.dpad_right) { // specimen arm goes down
            this.rail.setPosition(0.5); //0.42
            this.specimenArm.setPosition(0.15);
        }
        else if (gamepad2.right_trigger > 0.1) rotation.setPosition(0.65); // horizontal sample claw
        else if (gamepad2.left_trigger > 0.1) rotation.setPosition(0.2); // vertical-ish sample claw
        else if (gamepad2.a) bridge.setPosition(0.96); // drops sample arm all the way down
        else if (gamepad2.y) bridge.setPosition(0.5); // raises sample arm
        else if (gamepad2.b) bridge.setPosition(0.92); // "hover" mode
        else if (gamepad2.x) bridge.setPosition(0.85); // "backout" mode
        else if (gamepad2.dpad_left) bridge.setPosition(0.7); // back up specimen score
    }

}


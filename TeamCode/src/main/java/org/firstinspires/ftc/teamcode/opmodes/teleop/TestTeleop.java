package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="DO NOT USE THIS TELEOP") public class TestTeleop extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;private DcMotor frontRight;

    public Servo bridge;
    public Servo sampleClaw;
    public Servo specimenArm;
    public Servo specimenClaw;
    public Servo rail;
    public Servo rotation;
// public Servo ledIndicator;

    private IMU imu;
    public double DRIVE_POWER_VARIABLE = 1;
    public boolean specimenIsOpen;
    public boolean sampleIsOpen;
    public double specimenPosition;
    public double samplePosition;
    public ElapsedTime specimenTimer;
    public ElapsedTime sampleTimer;
// public ElapsedTime ledTimer;

    // ----- New fields for auto clipping routine -----
    private boolean autoClipActive = false; // whether auto clipping is running
    private int autoClipState = 0;          // state machine state
    private ElapsedTime autoTimer = new ElapsedTime();
    private boolean prevDpadDown = false;   // for edge-triggered toggle

    // Auto-cycle timing constants (in seconds)
    private static final double T_GOTO_PICKUP = 1.0;  // optional drive-to-(38,-59) wait (assumes you start there)
    private static final double T_GRAB = 0.5;         // time to close the sample claw
    private static final double T_DRIVE = 3.0;        // drive time from pickup to clipping location
    private static final double T_RELEASE = 0.5;      // time to open the claw (release sample)
    private static final double T_RETURN = 3.0;       // drive time from clipping location back to pickup
    private static final double AUTO_DRIVE_POWER = 0.5; // drive power multiplier for auto mode

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
        // initializing the imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        /* servo initialization */
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

        // ----- Check for auto clipping toggle (gamepad2.dpad_down) -----
        if (gamepad2.dpad_down && !prevDpadDown) {
            autoClipActive = !autoClipActive;  // toggle auto clipping
            if (!autoClipActive) {
                // If turning off auto mode, stop all drive motors and reset state.
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                autoClipState = 0;
            } else {
                // If starting auto mode, reset state machine timer and state.
                autoClipState = 0;
                autoTimer.reset();
            }
        }
        prevDpadDown = gamepad2.dpad_down;

        // ----- If auto clipping mode is active, override manual drive -----
        if (autoClipActive) {
            double autoX = 0.0;
            double autoY = 0.0;
            // Simple state machine for auto clipping cycle:
            // State 0: (Optional) Wait/drive to pickup position (assumed to be at (38, -59))
            // State 1: Close sample claw to grab sample.
            // State 2: Drive to clipping location (from (38, -59) to (-3, -31)).
            // State 3: Open sample claw to release sample.
            // State 4: Drive back to pickup position.
            // Then loop back to state 1.
            switch(autoClipState) {
                case 0:
                    // STATE 0: Ensure we are at pickup. Here we simply hold position.
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    if (autoTimer.seconds() >= T_GOTO_PICKUP) {
                        autoClipState = 1;
                        autoTimer.reset();
                    }
                    break;
                case 1:
                    // STATE 1: Grab sample by closing sample claw.
                    sampleClaw.setPosition(0.8);  // closed
                    if (autoTimer.seconds() >= T_GRAB) {
                        autoClipState = 2;
                        autoTimer.reset();
                    }
                    break;
                case 2:
                    // STATE 2: Drive to clipping location.
                    // Compute drive vector from (38,-59) to (-3,-31): Δx = -41, Δy = 28.
                    double magnitude = Math.sqrt(41*41 + 28*28);
                    double unitX = -41 / magnitude;
                    double unitY = 28 / magnitude;
                    autoX = unitX * AUTO_DRIVE_POWER;
                    autoY = unitY * AUTO_DRIVE_POWER;
                    // Use same mecanum calculation (no rotation) to set motor powers.
                    double denominator = Math.max(Math.abs(autoY) + Math.abs(autoX), 1);
                    double flPower = (autoY + autoX) / denominator;
                    double frPower = (autoY - autoX) / denominator;
                    double blPower = (autoY - autoX) / denominator;
                    double brPower = (autoY + autoX) / denominator;
                    frontLeft.setPower(flPower);
                    frontRight.setPower(frPower);
                    backLeft.setPower(blPower);
                    backRight.setPower(brPower);
                    if (autoTimer.seconds() >= T_DRIVE) {
                        autoClipState = 3;
                        autoTimer.reset();
                    }
                    break;
                case 3:
                    // STATE 3: Release sample by opening the claw.
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    sampleClaw.setPosition(0.0);  // open
                    if (autoTimer.seconds() >= T_RELEASE) {
                        autoClipState = 4;
                        autoTimer.reset();
                    }
                    break;
                case 4:
                    // STATE 4: Drive back to pickup position.
                    // Reverse the drive vector from state 2.
                    double unitXRev = -autoX; // reverse direction
                    double unitYRev = -autoY;
                    double autoXReturn = unitXRev * AUTO_DRIVE_POWER;
                    double autoYReturn = unitYRev * AUTO_DRIVE_POWER;
                    double denominatorReturn = Math.max(Math.abs(autoYReturn) + Math.abs(autoXReturn), 1);
                    double flReturn = (autoYReturn + autoXReturn) / denominatorReturn;
                    double frReturn = (autoYReturn - autoXReturn) / denominatorReturn;
                    double blReturn = (autoYReturn - autoXReturn) / denominatorReturn;
                    double brReturn = (autoYReturn + autoXReturn) / denominatorReturn;
                    frontLeft.setPower(flReturn);
                    frontRight.setPower(frReturn);
                    backLeft.setPower(blReturn);
                    backRight.setPower(brReturn);
                    if (autoTimer.seconds() >= T_RETURN) {
                        // Once returned, loop back to grab a new sample.
                        autoClipState = 1;
                        autoTimer.reset();
                    }
                    break;
            }
            // (Optional) add telemetry to indicate auto mode state.
            telemetry.addData("AutoClip Mode", "Active | State: " + autoClipState);
            telemetry.update();
            return; // skip the rest of the loop to avoid manual control interference
        }

        // ----- Manual teleop driving (only if autoClipActive is false) -----
        // Get drive inputs (negated Y because joystick Y is reversed)
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;  // joystick Y as is to match standard coordinate system
        double rx = -DRIVE_POWER_VARIABLE * (gamepad1.right_trigger - gamepad1.left_trigger);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-centric transformation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers for manual drive
        frontLeft.setPower(frontLeftPower * DRIVE_POWER_VARIABLE);
        frontRight.setPower(frontRightPower * DRIVE_POWER_VARIABLE);
        backLeft.setPower(backLeftPower * DRIVE_POWER_VARIABLE);
        backRight.setPower(backRightPower * DRIVE_POWER_VARIABLE);

        // ----- Other teleop controls -----
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
        else if (gamepad1.right_bumper) DRIVE_POWER_VARIABLE = 1; // normal speed
        else if (gamepad1.b && sampleTimer.milliseconds() > 250) sampleClaw.setPosition(0); // open sample claw
        else if (gamepad1.y && sampleTimer.milliseconds() > 250) sampleClaw.setPosition(0.8); // close sample claw
        else if (gamepad1.x) imu.resetYaw();

        // gamepad2 - intake systems (Kaichen)
        if (gamepad2.left_bumper && sampleTimer.milliseconds() > 250) { // toggle sample claw
            if (sampleIsOpen) {
                sampleClaw.setPosition(0.8);
                sampleIsOpen = false;
            } else {
                sampleClaw.setPosition(0.0);
                sampleIsOpen = true;
            }
            sampleTimer.reset();
        } else if (gamepad2.right_bumper && specimenTimer.milliseconds() > 250) { // toggle specimen claw
            if (specimenIsOpen) {
                specimenClaw.setPosition(1.0);
                specimenIsOpen = false;
            } else {
                specimenClaw.setPosition(0.0);
                specimenIsOpen = true;
            }
            specimenTimer.reset();
        } else if (gamepad2.dpad_up) { // specimen arm up
            this.rail.setPosition(0.61);
            this.specimenArm.setPosition(0.83);
        } else if (gamepad2.dpad_right) { // specimen arm down
            this.rail.setPosition(0.5);
            this.specimenArm.setPosition(0.15);
        }
        else if (gamepad2.right_trigger > 0.1) rotation.setPosition(0.65);
        else if (gamepad2.left_trigger > 0.1) rotation.setPosition(0.2);
        else if (gamepad2.a) bridge.setPosition(0.96);
        else if (gamepad2.y) bridge.setPosition(0.5);
        else if (gamepad2.b) bridge.setPosition(0.92);
        else if (gamepad2.x) bridge.setPosition(0.85);
        else if (gamepad2.dpad_left) bridge.setPosition(0.6);
    }
}
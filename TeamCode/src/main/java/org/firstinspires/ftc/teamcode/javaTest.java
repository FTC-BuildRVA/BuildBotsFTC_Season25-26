package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// FTC DASHBOARD IMPORTS
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@TeleOp(name = "test (Blocks to Java)")
@Config
public class javaTest extends LinearOpMode {

    //FTC DASHBOARD
    private FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    public static double max = 3000;
    public static double min = 1000;

    // PHONE TELEMETRY FOR CAMERA
    List<String> telemetryStrings = new ArrayList<>();

    private IMU imuIMU;
    private DcMotor intake;
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private CRServo feederLeft;
    private CRServo feederMid;
    private CRServo feederRight;

    int relative;
    double aimError;
    double aimI;
    boolean gl;
    int Shooter;
    int minRpm;
    int maxRpm;
    double shooterRpm;
    boolean USE_WEBCAM;
    AprilTagDetection myAprilTagDetection;
    double Range2;
    List<AprilTagDetection> myAprilTagDetections;
    double turn;
    float forwardRelative;
    float strafeRelative;
    ElapsedTime myElapsedTime;
    double moveSpeedMultiplier;
    double lastError;
    double forward;
    double aimMoveSpeed;
    AprilTagProcessor myAprilTagProcessor;
    double angleBearingGoal;
    double MAX;
    double strafe;
    VisionPortal myVisionPortal;
    int okAngle;
    double angleYawGoal;
    double aimYawMultiplier;
    double aimP;
    float turnRelative;
    int moveDirection;
    long newTime;
    long oldTime;
    double aimD;
    boolean shooterOver;
    double intake2;
    int cameraState;
    int shooterStep;
    double maxInt;
    int shooterEnable;

    double oldIntake2 = 0;
    boolean intakeModeFlag = false;

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();

        imuIMU = hardwareMap.get(IMU.class, "imu");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooter_left");
        shooterRight = hardwareMap.get(DcMotor.class, "shooter_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        feederLeft = hardwareMap.get(CRServo.class, "feeder_left");
        feederMid = hardwareMap.get(CRServo.class, "feeder_mid");
        feederRight = hardwareMap.get(CRServo.class, "feeder_right");

        // Put initialization blocks here.
        USE_WEBCAM = true;
        setup();
        initAprilTag();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Get the current time in milliseconds. The value returned represents
                // the number of milliseconds since midnight, January 1, 1970 UTC.
                newTime = System.currentTimeMillis();
                getDetections();
                game();
                driveRelative();
                aim();
                process();
//                if (gamepad1.aWasPressed()) {
//
//                }
                telemetry2();
                telemetryAprilTag();
                telemetry.update();
                dashTelemetry();
                oldTime = newTime;
            }
        }

        // CLEANUP
        if (myVisionPortal != null) {
            myVisionPortal.close();
        }
    }

    /**
     * This takes field relative, converts to field relative and drives the robot
     */
    private void driveRelative() {
        double theta;
        double r;

        theta = Math.atan2(forwardRelative, strafeRelative) / Math.PI * 180;
        r = Math.sqrt(forwardRelative * forwardRelative + strafeRelative * strafeRelative);
        theta = AngleUnit.DEGREES.normalize(theta - imuIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        if (relative == 1) {
            forward = r * Math.sin(theta / 180 * Math.PI);
            turn = turnRelative;
            strafe = r * Math.cos(theta / 180 * Math.PI);
        } else {
            forward = forwardRelative;
            strafe = strafeRelative;
            turn = turnRelative;
        }
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.vvvvvvvvvv
     */
    private void telemetryAprilTag() {
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetectionItem : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetectionItem;
            // Display info about the detection.
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    /**
     * Describe this function...
     */
    private void setup() {
        relative = 0;
        Range2 = 0;
        myElapsedTime = new ElapsedTime();
        aimMoveSpeed = 0.75;
        okAngle = 1;
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MAX = 0.75;
        moveSpeedMultiplier = MAX;
        moveDirection = 1;
        maxInt = 0.75;
        intake2 = 0;
        Shooter = 0;
        shooterOver = false;
        shooterEnable = 1;
        aimMoveSpeed = 0.75;
        aimYawMultiplier = 0.1;
        aimP = 0;
        aimI = 0;
        aimD = 0;
        // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics Control
        // Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction
        // that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imuIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        shooterStep = 50;
        maxRpm = 2500;
        minRpm = 1900;
        lastError = 0;
        feederLeft.setDirection(CRServo.Direction.FORWARD);
        feederMid.setDirection(CRServo.Direction.FORWARD);
        feederRight.setDirection(CRServo.Direction.REVERSE);
    }

    /**
     * Initialize AprilTag Detection.vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
     */
    private void initAprilTag() {
        telemetryHelper("Camera initializing...");
        // Create the AprilTag processor by using a builder.
        myAprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Create the webcam vision portal by using a builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(800, 448))
                .build();
        // Supported resolutions:
        // 640x360
        // 640x480
        // 800x448
        // 864x480
        // 800x600
        // 1920x1080
        telemetryHelper("Camera initialized!");
        telemetryHelper("Waiting for camera stream to start...");

        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetryHelper("Camera streaming!");
        }
        else {
            telemetryHelper("Camera already streaming!");
        }

        sleep(1000);

        // BEGIN EXPOSURE, GAIN, FOCUS MODIFICATION
        if (!isStopRequested()) {
            telemetryHelper("Getting exposure control object...");
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            telemetryHelper("Setting manual exposure control...");
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
                telemetryHelper("Set manual exposure control!");
            }
            else{
                telemetryHelper("Manual exposure control already set!");
            }

            telemetryHelper("Setting exposure...");
            exposureControl.setExposure((long) 2, TimeUnit.MILLISECONDS);
            sleep(20);
            telemetryHelper("Exposure set!");

            telemetryHelper("Getting gain control object...");
            GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
            telemetryHelper("Got gain control object!");

            telemetryHelper("Setting gain...");
            gainControl.setGain(100);
            sleep(20);
            telemetryHelper("Gain set!");

            telemetryHelper("Getting focus control object...");
            FocusControl focusControl = myVisionPortal.getCameraControl(FocusControl.class);
            telemetryHelper("Got focus control object!");
            telemetryHelper("Setting manual focus control...");
            if (focusControl.getMode() != FocusControl.Mode.Fixed) {
                focusControl.setMode(FocusControl.Mode.Fixed);
                sleep(50);
                telemetryHelper("Set manual focus control!");
            }
            else{
                telemetryHelper("Manual exposure control already set!");
            }
            telemetryHelper("Setting focus...");
            focusControl.setFocusLength(25);
            sleep(20);
            telemetryHelper("Focus set!");
        }
        // END EXPOSURE AND GAIN MODIFICATION
        telemetryHelper("Camera ready!");
    }

    /**
     * Describe this function...
     */
    private void aim() {
        PIDFCoefficients aimPID;

        aimError = -(angleBearingGoal + angleYawGoal * aimYawMultiplier);
        if (gamepad1.right_bumper) {
            moveSpeedMultiplier = aimMoveSpeed;
//            aimPID = new PIDFCoefficients(0.02, 0.001, 0.01, 0);
            aimPID = new PIDFCoefficients(0.02, 0.01, 0.01, 0);
            aimP = aimError * aimPID.p;
//            aimI = aimI + aimError * aimPID.i;
            aimI = aimI + aimPID.i * (aimError / Math.abs(aimError));
            if (Math.abs(aimError) < okAngle){
                aimI = 0;
            }
            aimD = (aimError - lastError) * aimPID.d;
            turn = turnRelative + aimP + aimI + aimD;


//            if (Math.abs(aimError) < 3){
//                frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                turn=0;
//            }

        } else {
            moveSpeedMultiplier = MAX;
            aimI = 0;
        }
        lastError = aimError;
    }

    /**
     * Describe this function...
     */
    private void getDetections() {
        gl = false;
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        for (AprilTagDetection myAprilTagDetectionItem2 : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetectionItem2;
            if (myAprilTagDetection.id == 20) {
                angleBearingGoal = myAprilTagDetection.ftcPose.bearing;
                angleYawGoal = myAprilTagDetection.ftcPose.yaw;
                Range2 = myAprilTagDetection.ftcPose.range;
                gl = true;
            }
        }
    }

    /**
     * Describe this function...
     */
    private void game() {
        gamepad1.rumble(0, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        if (gl) {
            gamepad1.rumble(0, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (gamepad1.right_bumper) {
            if (Math.abs(aimError) < okAngle) {
                gamepad1.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }
        } else {
        }
        turnRelative = gamepad1.right_stick_x;
        forwardRelative = -gamepad1.left_stick_y;
        strafeRelative = gamepad1.left_stick_x;
        if (gamepad1.rightStickButtonWasPressed()) {
            intake2 = 0;
            feederLeft.setPower(0);
            feederMid.setPower(0);
            feederRight.setPower(0);
        }
        if (gamepad1.psWasPressed()) {
            if (shooterOver) {
                shooterOver = false;
            } else {
                shooterOver = true;
            }
        }
        if (shooterOver) {
            if (gamepad1.dpadLeftWasPressed()) {
                Shooter += shooterStep * -1;
            }
            if (gamepad1.dpadRightWasPressed()) {
                Shooter += shooterStep;
            }
        } else {
            if (Range2 > 117.6) {
                Shooter = 2100;
            } else {
                Shooter = 1950;
            }
        }
        // INTAKE CONTROL
        if (gamepad1.b || gamepad1.right_bumper) {
            if (!intakeModeFlag) {
                oldIntake2 = intake2;
                intakeModeFlag = true;
            }

            if (gamepad1.dpad_down) {
                intake2 = (maxInt * -1.0);
            }
            else if (gamepad1.dpad_up) {
                intake2 = (maxInt);
            }
            else {
                intake2 = 0.0;
            }
        }
        else{
            if (intakeModeFlag) {
                intake2 = oldIntake2;
                intakeModeFlag = false;
            }
            if (gamepad1.dpadDownWasPressed()) {
                intake2 = (maxInt * -1.0);
            }
            if (gamepad1.dpadUpWasPressed()) {
                intake2 = (maxInt);
            }
        }
        if (shooterRpm < Shooter - 200) {
            myElapsedTime.reset();
        }
        if (myElapsedTime.milliseconds() < 1000) {
//            feederRight.setPower(0);
//            feederLeft.setPower(0);
            feederMid.setPower(0);
        } else {
            if (gamepad1.b) {
                feederLeft.setPower(1);
                feederMid.setPower(1);
                feederRight.setPower(1);
            } else {
                feederLeft.setPower(0);
                feederMid.setPower(0);
                feederRight.setPower(0);
            }
        }
        if (gamepad1.a) {
            feederLeft.setPower(1);
            feederRight.setPower(1);
        }
        if (gamepad1.start) {
            imuIMU.resetYaw();
        }
        if (gamepad1.squareWasPressed()) {
            moveDirection = moveDirection * -1;
        }
        if (gamepad1.shareWasPressed()) {
            if (relative == 1) {
                relative = 0;
            } else {
                imuIMU.resetYaw();
                relative = 1;
            }
        }
    }

    /**
     * Describe this function...
     */
    private void process() {
        shooterRpm = Math.abs(((DcMotorEx) shooterLeft).getVelocity()) * 2.1429;
        turn = turn * moveSpeedMultiplier;
        forward = forward * moveSpeedMultiplier * moveDirection;
        strafe = strafe * moveSpeedMultiplier * moveDirection;
        ((DcMotorEx) shooterLeft).setVelocity((Shooter / 2.1429) * shooterEnable);
        ((DcMotorEx) shooterRight).setVelocity((Shooter / 2.1429) * shooterEnable);
        frontLeft.setPower(forward + strafe + turn);
        frontRight.setPower((forward - strafe) - turn);
        backLeft.setPower((forward - strafe) + turn);
        backRight.setPower((forward + strafe) - turn);
        intake.setPower(intake2);
    }

    /**
     * Describe this function...
     */
    private void telemetry2() {
        telemetry.addData("Loop frequency (Hz)", Double.parseDouble(JavaUtil.formatNumber((1 / (newTime - oldTime)) * 1000, 1)));
        telemetry.addData("imu", imuIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("shooter rpm", Double.parseDouble(JavaUtil.formatNumber(shooterRpm, 0)));
        telemetry.addData("bearing angle", Double.parseDouble(JavaUtil.formatNumber(angleBearingGoal, 2)));
        telemetry.addData("yaw angle", Double.parseDouble(JavaUtil.formatNumber(angleYawGoal, 2)));
        telemetry.addData("aim error", Double.parseDouble(JavaUtil.formatNumber(aimError, 2)));
        telemetry.addData("target shooter rpm", Double.parseDouble(JavaUtil.formatNumber(Shooter, 0)));
        telemetry.addData("intake speed", Double.parseDouble(JavaUtil.formatNumber(intake2, 2)));
        telemetry.addData("over", shooterOver);
    }

    private void dashTelemetry() {
        packet.put("Max RPM", 3000);
        packet.put("Min RPM", 1000);
        packet.put("Target RPM", Shooter);
        packet.put("Left RPM", shooterRpm);
        packet.put("Right RPM",Math.abs(((DcMotorEx) shooterRight).getVelocity()) * 2.1429);
        dashboard.sendTelemetryPacket(packet);
    }

    private void telemetryHelper(String str) {
        telemetryStrings.add(str);
        for(String telStr : telemetryStrings){
            telemetry.addLine(telStr);
        }
        telemetry.update();
    }
}
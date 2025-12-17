/**
 * This program is used to play with and find useful settings for any camera to best detect AprilTags
 */

// PACKAGE STATEMENT
package org.firstinspires.ftc.teamcode.Utility_CameraCalibration;

// CORE FTC IMPORTS

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// MISCELLANEOUS IMPORTS
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.annotation.SuppressLint;
import android.util.Size;

import java.util.concurrent.TimeUnit;

// VISION ANALYSIS FTC IMPORTS
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// DASHBOARD ACMEROBOTICS IMPORTS
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;

/*
 * Code starts here
 */
@Config
@TeleOp(name = "Utility_CameraCalibration", group = "Linear OpMode")
public class Main extends LinearOpMode {

    // CAMERA & VISION VARIABLES
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public static int myExposure = 100;
    private int lastMyExposure;
    private int minExposure;
    private int maxExposure;

    public static int myGain = 100;
    private int lastMyGain;
    private int minGain;
    private int maxGain;

    public static int myWhiteBal = 4000;
    private int lastWhiteBal;
    private int minWhiteBal;
    private int maxWhiteBal;

    public static int myFocus = 50;
    private int lastMyFocus;
    private int minFocus;
    private int maxFocus;

    public static int myPan = 0;
    private int lastMyPan;
    private int minPan;
    private int maxPan;

    public static int myTilt = 0;
    private int lastMyTilt;
    private int minTilt;
    private int maxTilt;

    public static int myZoom = 100;
    private int lastMyZoom;
    private int minZoom;
    private int maxZoom;

    public static boolean autoExposureGain = false;
    private boolean lastAutoExposureGain = autoExposureGain;

    public static boolean autoWhiteBalance = false;
    private boolean lastAutoWhiteBalance = autoWhiteBalance;

    public static boolean autoFocus = false;
    private boolean lastAutoFocus = autoFocus;

    // TELEMETRY VARIABLES
    private FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    public static boolean enableDashboardCameraStream = false;

    // MISCELLANEOUS VARIABLES
    private ElapsedTime runtime = new ElapsedTime();
    public static int dashboardCameraFramerate = 1;
    private int lastDashboardCameraFramerate = dashboardCameraFramerate;

    // OP MODE
    @Override
    public void runOpMode() {
        // Print instructions to the driver hub
        handleInstructions();

        // Setups
        initializeCamera();
        getCameraSetting();
        initializeDashboard();

        // Wait here until the PLAY button is pressed on the driver      hub
        waitForStart();
        runtime.reset();

        // Loops until the STOP button is pressed on the driver hub
        while (opModeIsActive()) {
            if (enableDashboardCameraStream) {
                handleFramerateChanges();
            }
            handleCameraSettingsChanges();
            handleTelemetry();
        }
    }

    /**
     * Prints instructions to the driver hub telemetry screen, since most of the magic happens on the FTC Dashboard
     */
    void handleInstructions() {
        telemetry.addLine("An Opmode to Calibrate the Camera.");
        telemetry.addLine("- - -");
        telemetry.addLine("On a computer, connect to the Robot's wifi network, then go to 192.168.43.1:8080/dash");
        telemetry.addLine("\nThen, press PLAY on the driver hub to begin!");
        telemetry.addLine("- - -");
        telemetry.addLine("To view the camera feed, download https://github.com/Genymobile/scrcpy and launch scrcpy.exe");

        telemetry.update();
    }

    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        if (enableDashboardCameraStream) {
            dashboard.startCameraStream(visionPortal, dashboardCameraFramerate);
        }
    }

    private void initializeCamera() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the webcam vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();

        // Supported resolutions:
        // 640x360
        // 640x480
        // 800x448
        // 864x480
        // 800x600
        // 1920x1080
    }

    /**
     * Sends telemetry data to the FTC Dashboard
     */
    @SuppressLint("DefaultLocale")
    void handleTelemetry() {
        // Read sensors
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Empty the packet
        packet.clearLines();

        // Fill the packet
        packet.put("Voltage", String.format("%.2f V", voltage));
        packet.put("Max exposure", maxExposure);
        packet.put("Min exposure", minExposure);
        packet.put("Max gain", maxGain);
        packet.put("Min gain", minGain);
        packet.put("Max white balance", maxWhiteBal);
        packet.put("Min white balance", minWhiteBal);
        packet.put("Max focus", maxFocus);
        packet.put("Min focus", minFocus);
        packet.put("Max pan", maxPan);
        packet.put("Min pan", minPan);
        packet.put("Max tilt", maxTilt);
        packet.put("Min tilt", minTilt);
        packet.put("Max zoom", maxZoom);
        packet.put("Min zoom", minZoom);
        packet.put("Dashboard Framerate", dashboardCameraFramerate);
        packet.clearLines();

        // Send the telemetry
        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * If the user requests a different framerate on the FTCDashboard, we need to
     * restart the camera stream for it to take effect.
     * -
     * Faster framerates will make the main loop run slower, since it needs to send
     * the video feed to the dashboard. This means asking to view a lower framerate stream
     * will make your robot more responsive and better at detecting AprilTags.
     */
    private void handleFramerateChanges() {
        if (dashboardCameraFramerate != lastDashboardCameraFramerate) {
            // Stop the camera stream (if applicable)
            try {
                dashboard.stopCameraStream();
            } catch (Exception e) {
                // do nothing
            }
            // Start the camera stream with the new framerate value
            dashboard.startCameraStream(visionPortal, dashboardCameraFramerate);
            lastDashboardCameraFramerate = dashboardCameraFramerate;
        }
    }

    /**
     * Read this camera's minimum and maximum Exposure and Gain settings.
     * Can only be called AFTER calling initializeCamera();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();

            WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
            minWhiteBal = whiteBalanceControl.getMinWhiteBalanceTemperature();
            maxWhiteBal = whiteBalanceControl.getMaxWhiteBalanceTemperature();

            FocusControl focusControl = visionPortal.getCameraControl(FocusControl.class);
            minFocus = (int) focusControl.getMinFocusLength();
            maxFocus = (int) focusControl.getMaxFocusLength();

            PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
            PtzControl.PanTiltHolder holderMaxPT = ptzControl.getMaxPanTilt();
            minPan = 0;
            maxPan = holderMaxPT.pan;
            minTilt = 0;
            maxTilt = holderMaxPT.tilt;
            minZoom = ptzControl.getMinZoom();
            maxZoom = ptzControl.getMaxZoom();
        }
    }

    /**
     * Adjust camera settings based on user variables set in the FTC Dashboard
     */
    private void handleCameraSettingsChanges() {
        boolean settingsChanged = false;

        // Check if auto mode settings have changed
        if (autoExposureGain != lastAutoExposureGain) {
            lastAutoExposureGain = autoExposureGain;
            settingsChanged = true;
        }

        if (autoWhiteBalance != lastAutoWhiteBalance) {
            lastAutoWhiteBalance = autoWhiteBalance;
            settingsChanged = true;
        }

        if (autoFocus != lastAutoFocus) {
            lastAutoFocus = autoFocus;
            settingsChanged = true;
        }

        // Check if any values have changed and clip them
        if (myExposure != lastMyExposure) {
            myExposure = Range.clip(myExposure, minExposure, maxExposure);
            lastMyExposure = myExposure;
            settingsChanged = true;
        }

        if (myGain != lastMyGain) {
            myGain = Range.clip(myGain, minGain, maxGain);
            lastMyGain = myGain;
            settingsChanged = true;
        }

        if (myWhiteBal != lastWhiteBal) {
            myWhiteBal = Range.clip(myWhiteBal, minWhiteBal, maxWhiteBal);
            lastWhiteBal = myWhiteBal;
            settingsChanged = true;
        }

        if (myFocus != lastMyFocus) {
            myFocus = Range.clip(myFocus, minFocus, maxFocus);
            lastMyFocus = myFocus;
            settingsChanged = true;
        }

        if (myPan != lastMyPan) {
            myPan = Range.clip(myPan, minPan, maxPan);
            lastMyPan = myPan;
            settingsChanged = true;
        }

        if (myTilt != lastMyTilt) {
            myTilt = Range.clip(myTilt, minTilt, maxTilt);
            lastMyTilt = myTilt;
            settingsChanged = true;
        }

        if (myZoom != lastMyZoom) {
            myZoom = Range.clip(myZoom, minZoom, maxZoom);
            lastMyZoom = myZoom;
            settingsChanged = true;
        }

        // If any setting changed, apply all settings at once
        if (settingsChanged) {
            setNewCameraSettings(myExposure, myGain, myWhiteBal, myFocus, myPan, myTilt, myZoom);
        }
    }

    /**
     * Apply all camera settings at once.
     * Sets mode to auto or manual based on user preferences, then applies manual values if in manual mode.
     * Can only be called AFTER calling initAprilTag();
     * Returns true if controls are set successfully.
     */
    private boolean setNewCameraSettings(int exposureMS, int gain, int whiteBalance, double focusLength, int pan, int tilt, int zoom) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        if (!isStopRequested()) {
            // Handle exposure and gain mode
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (autoExposureGain) {
                // Set to auto mode (AperturePriority for C920/C270)
                if (exposureControl.getMode() != ExposureControl.Mode.AperturePriority) {
                    exposureControl.setMode(ExposureControl.Mode.AperturePriority);
                    sleep(50);
                }
            } else {
                // Set to manual mode and apply values
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }

            // Handle white balance mode
            WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
            if (autoWhiteBalance) {
                // Set to auto mode
                if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.AUTO) {
                    whiteBalanceControl.setMode(WhiteBalanceControl.Mode.AUTO);
                    sleep(50);
                }
            } else {
                // Set to manual mode and apply value
                if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
                    whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
                    sleep(50);
                }
                whiteBalanceControl.setWhiteBalanceTemperature(whiteBalance);
                sleep(20);
            }

            // Handle focus mode
            FocusControl focusControl = visionPortal.getCameraControl(FocusControl.class);
            if (autoFocus) {
                // Set to auto mode (ContinuousAuto for C920)
                if (focusControl.getMode() != FocusControl.Mode.ContinuousAuto) {
                    focusControl.setMode(FocusControl.Mode.ContinuousAuto);
                    sleep(50);
                }
            } else {
                // Set to manual mode (Fixed) and apply value
                if (focusControl.getMode() != FocusControl.Mode.Fixed) {
                    focusControl.setMode(FocusControl.Mode.Fixed);
                    sleep(50);
                }
                focusControl.setFocusLength(focusLength);
                sleep(20);
            }

            // Handle PTZ (Pan, Tilt, Zoom) - always manual control
            PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
            ptzControl.setZoom(zoom);
            sleep(20);
            PtzControl.PanTiltHolder holderPT = new PtzControl.PanTiltHolder();
            holderPT.pan = pan;
            holderPT.tilt = tilt;
            ptzControl.setPanTilt(holderPT);
            sleep(20);


            return true;
        } else {
            return false;
        }
    }
}
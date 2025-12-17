/**
 * This program is used to play with and find useful settings for any camera to best detect AprilTags
 */

// PACKAGE STATEMENT
package org.firstinspires.ftc.teamcode.Utility_CameraCalibration;

// CORE FTC IMPORTS

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// MISCELLANEOUS FTC IMPORTS
import com.qualcomm.robotcore.util.ElapsedTime;

// VISION ANALYSIS FTC IMPORTS
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// DASHBOARD ACMEROBOTICS IMPORTS
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

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
            if(enableDashboardCameraStream){
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
        if(enableDashboardCameraStream){
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
                .build();
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
        packet.put("Voltage:", String.format("%.2f V", voltage));
        packet.put("Max exposure", maxExposure);
        packet.put("Min exposure", minExposure);
        packet.put("Max gain", maxGain);
        packet.put("Min gain", minGain);
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
            try{
                dashboard.stopCameraStream();
            }
            catch(Exception e){
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
        }
    }

    /**
     * Adjust camera settings based on user variables set in the FTC Dashboard
     */
    private void handleCameraSettingsChanges(){
        // Only adjust the exposure if the value has changed since we last checked
        if (myExposure != lastMyExposure) {
            myExposure = Range.clip(myExposure, minExposure, maxExposure);
            setManualExposure(myExposure, myGain);
            lastMyExposure = myExposure;
        }
        // Only adjust the gain if the value has changed since we last checked
        if (myGain != lastMyGain) {
            myGain = Range.clip(myGain, minGain, maxGain);
            setManualExposure(myExposure, myGain);
            lastMyGain = myGain;
        }
    }


    /**
     * Manually set the camera gain and exposure.
     * Can only be called AFTER calling initAprilTag();
     * Returns true if controls are set.
     */
    private boolean setManualExposure(int exposureMS, int gain) {
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
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return true;

        } else {
            return false;
        }
    }
}

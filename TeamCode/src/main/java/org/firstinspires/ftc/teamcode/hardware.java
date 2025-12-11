package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public class hardware {

    public DcMotor FrontLeft;
    public DcMotor BackLeft;
    public DcMotor FrontRight;
    public DcMotor BackRight;
    public IMU     imu         = null;      // Control/Expansion Hub IMU



    public static final double TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 3.77953;

    public int init(HardwareMap hwMp){
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        FrontLeft = hwMp.get(DcMotor.class,"front_left");
        BackLeft = hwMp.get(DcMotor.class,"back_left");
        FrontRight = hwMp.get(DcMotor.class,"front_right");
        BackRight = hwMp.get(DcMotor.class,"back_right");

        imu = hwMp.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        return 0;
    }
}

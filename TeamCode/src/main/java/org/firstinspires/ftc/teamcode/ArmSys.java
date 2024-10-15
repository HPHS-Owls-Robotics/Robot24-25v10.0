package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class ArmSys {
    DcMotor Slide;
    CRServo slide;
    CRServo sweep;


    public ArmSys(HardwareMap hardwareMap)
    {}
    public void sweep(int sleep)
    {
        sweep.setPower(1);
    }
    public void SlideToRung(int inches)
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setTargetPosition(-500);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(-0.5);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
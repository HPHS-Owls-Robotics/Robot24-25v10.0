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
    {
        Slide = hardwareMap.dcMotor.get("S_Motor");
        slide = hardwareMap.crservo.get("slide");
        sweep = hardwareMap.crservo.get("sweep");
    }
    public void sweepIn()
    {
        sweep.setPower(-3);
    }
    public void sweepOut()
    {
        sweep.setPower(3);
    }
    public void sweepOff()
    {
        sweep.setPower(0);
    }
    public void SlideToRung()
    {
        Slide(-1500,-0.5f);

        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setTargetPosition(-1500);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(-0.5);
    }
    public void slideExtend()
    {
        slide.setPower(3);
    }
    public void SlideToBasket()
    {
        Slide(-3300,-0.5f);
    }
    public void SlideDown()
    {
        Slide(-0,-0.5f);
    }
    private void Slide(int ticks, float power)
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setTargetPosition(ticks);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);
    }
}
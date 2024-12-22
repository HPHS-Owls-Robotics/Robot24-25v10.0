package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "uhhh")
public final class WHATISHAPPENING extends LinearOpMode {

    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;

    public void runOpMode() throws InterruptedException {
        //initialization
        FRMotor = hardwareMap.dcMotor.get("FR_Motor");
        FLMotor = hardwareMap.dcMotor.get("FL_Motor");
        BLMotor = hardwareMap.dcMotor.get("BL_Motor");
        BRMotor = hardwareMap.dcMotor.get("BR_Motor");
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            FLMotor.setPower(-0.5);
            FRMotor.setPower(0.5);
            BRMotor.setPower(-0.5);
            BLMotor.setPower(0.5);
        }

        telemetry.addData("NO", FLMotor.getCurrentPosition());
        telemetry.addData("NO", FRMotor.getCurrentPosition());
        telemetry.addData("NO", BRMotor.getCurrentPosition());
        telemetry.addData("NO", BLMotor.getCurrentPosition());
            telemetry.update();


    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "three")
public final class three extends LinearOpMode {

    MecanumDrive drive;
    ArmSys arm;
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(24, 60, 0);
        drive = new MecanumDrive(hardwareMap, beginPose);
        arm = new ArmSys(hardwareMap);
        waitForStart();
        drive = new MecanumDrive(hardwareMap, beginPose);
        arm = new ArmSys(hardwareMap);
        waitForStart();
        //arm.SlideToBasket();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(45, 60))



                        .build());


    }
}
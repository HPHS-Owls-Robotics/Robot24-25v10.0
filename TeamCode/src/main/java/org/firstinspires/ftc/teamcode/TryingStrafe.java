package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "aa")
public final class TryingStrafe extends LinearOpMode {

    MecanumDrive drive;
    ArmSys arm;
    MoveSysOdo o;
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(32, 60, 0);// CHANGE HEADING(3rd number) TO 90 DEGREES AND RECHECK COORDINATES
        drive = new MecanumDrive(hardwareMap, beginPose);
        arm = new ArmSys(hardwareMap);
        waitForStart();
        //arm.SlideToBasket();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(60,60))
                        .build());
        sleep(1000);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(40,60))
                        .build());
        sleep(1000);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(40,32))
                        .build());
        sleep(1000);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(40,60))
                        .build());
        sleep(1000);

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(32,60))
                        .build());
        sleep(1000);



    }
}
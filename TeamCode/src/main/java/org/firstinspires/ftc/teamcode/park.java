package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "park")
public final class park extends LinearOpMode {

    MecanumDrive drive;
    ArmSys arm;
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, beginPose);
        arm = new ArmSys(hardwareMap);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(10, 0))
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(10, 0))
                        .build());



//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        //.lineToY(20)
//                        .strafeTo(new Vector2d(-10, 0))
//                        .build());
//        telemetry.addData("First " , "move");
//        telemetry.update();
        // sleep(500);
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        //.lineToY(20)
//                        .strafeTo(new Vector2d(0,-28))
//                        .build());
//
//
        telemetry.addData("First " , "move");
        telemetry.update();
        arm.sweepIn();
        sleep(1000);
        arm.sweepOff();
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        //.lineToY(20)
//                        .strafeTo(new Vector2d(0, 0))
//                        .strafeTo(new Vector2d(20, 0))
//                        .build());
//        telemetry.addData("second " , "move");
//        telemetry.update();
//

        sleep(1000);
        arm.SlideToBasket();
        sleep(2000);
        arm.sweepOut();
        sleep(2000);


    }
}
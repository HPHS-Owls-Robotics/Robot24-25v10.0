package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "aletsgo")
public final class tryAuto extends LinearOpMode {

    MecanumDrive drive;
    ArmSys arm;
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(24, 60, 0);
        drive = new MecanumDrive(hardwareMap, beginPose);
        arm = new ArmSys(hardwareMap);
            waitForStart();
        //arm.SlideToBasket();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(45, 60))



                        .build());
        Pose2d poseOne = new Pose2d(45, 60, 0);

//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        //.lineToY(20)
//                        .strafeTo(new Vector2d(24, 60))
//                        //.strafeTo(new Vector2d(24, 60))
//                        .build());
            arm.SlideToBasket();
            sleep(2500);
            arm.sweepOut();
            sleep(2500);
            arm.SlideDown();
            arm.sweepOff();
            sleep(3000);

        Actions.runBlocking(
                drive.actionBuilder(poseOne)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(36, 60))
                        .build());
        arm.sweepIn();
        arm.sweepOut();
        arm.sweepOff();
        sleep(2500);
        telemetry.addData("hello", "hello");
        telemetry.update();
        Pose2d poseTwo = new Pose2d(36, 60, 0);

        Actions.runBlocking(
                drive.actionBuilder(poseTwo)
                        //.lineToY(20)
                        .strafeTo(new Vector2d(36, 12))
                        .build());
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        //.lineToY(48)
//                        .strafeTo(new Vector2d(24, 40))
//                        .build());
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        //.lineToY(20)
//                        .strafeTo(new Vector2d(24, 60))
//                        .build());
//
//        sleep(2000);
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .lineToY(24)
//                        .build());
//
//
//        arm.sweepIn();
//        sleep(2500);
//        arm.sweepOff();
////        Actions.runBlocking(
////                drive.actionBuilder(beginPose)
////                        //.lineToY(20)
////                        .strafeTo(new Vector2d(60, 60))
////                        .build());
//        arm.SlideToBasket();
//        sleep(2500);
//        arm.sweepOut();
//        sleep(2500);
//        arm.sweepOff();

//            arm.SlideDown();
//            sleep(2000);
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
//        telemetry.addData("First " , "move");
//        telemetry.update();
//            arm.sweepIn();
//            sleep(1000);
//            arm.sweepOff();
////        Actions.runBlocking(
////                drive.actionBuilder(beginPose)
////                        //.lineToY(20)
////                        .strafeTo(new Vector2d(0, 0))
////                        .strafeTo(new Vector2d(20, 0))
////                        .build());
////        telemetry.addData("second " , "move");
////        telemetry.update();
////
//
//        sleep(1000);
//        arm.SlideToBasket();
//        sleep(2000);
//        arm.sweepOut();
//        sleep(2000);



    }
}
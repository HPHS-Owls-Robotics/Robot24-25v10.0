package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "one")
public final class one extends LinearOpMode {

    MecanumDrive drive;
    ArmSys arm;
    MoveSysOdo o;
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
                        .strafeTo(new Vector2d(-36, 60))
                        .build());
//        //arm.sweepIn();
//        arm.sweepOut();
//        arm.sweepOff();
//        sleep(2500);
//        telemetry.addData("hello", "hello");
//        telemetry.update();
//        o = new MoveSysOdo(hardwareMap);
//        o.right(36);
//        arm.sweepIn();
//        sleep(3000);
//        //o.right(-36);





    }
}
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name=" Close")
public class AutoTestClose extends LinearOpMode {

    int tag=0;

    int color =0;
    int sleepTimer =3000;
    int scanTimer=3000;
    MoveSysOdo m;


    @Override
    public void runOpMode() throws InterruptedException {
        m= new MoveSysOdo(hardwareMap);
        telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", m.getCalibrationStatus());
        telemetry.update();


        waitForStart();

        if(opModeIsActive()) {
            m.forward(54);

        }



    }

}
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Far")
public class AutoTestFar extends LinearOpMode {

    int tag=0;

    int color =0;
    int sleepTimer =3000;
    int scanTimer=3000;
    MoveSys m;


    @Override
    public void runOpMode() throws InterruptedException {
        m= new MoveSys(hardwareMap);
        telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", m.getCalibrationStatus());
        telemetry.update();


//        encoderDrive(DRIVE_SPEED,  54,  54, 5.0);
//        rotate(-90, TURN_SPEED);
//        encoderDrive(DRIVE_SPEED,  52,  52, 5.0);
//        rotate(90, TURN_SPEED);
//        encoderDrive(DRIVE_SPEED,  12,  12, 5.0);
//        rotate(80, TURN_SPEED);
//        encoderDrive(DRIVE_SPEED,  50,  50, 5.0);
//        rotate(80, TURN_SPEED);
//        encoderDrive(DRIVE_SPEED,  120,  120, 15.0);
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
        waitForStart();
        if(opModeIsActive())


            waitForStart();
        m.forward(78);
        sleep(3000);
        m.rotate(-90);
        sleep(3000);

        m.forward(52);
        sleep(3000);

        m.rotate(90);
        sleep(3000);

        m.forward(12);
        sleep(3000);

        m.rotate(80);
        sleep(3000);

        m.forward(50);
        sleep(3000);

        m.rotate(80);
        sleep(3000);

        m.forward(120);
        sleep(3000);

    }
//    public void findTag(int tag)
//    {
//        if(color!=1)
//        {
//            tag+=3;
//        }
//        if( a.getTag()== tag+3)
//        {
//            driveToTag();
//        }
//        else if(a.getTag()< tag)
//        {
//            m.rotate(90);
//            m.forward(-10 );
//            m.rotate(90);
//            findTag(tag);
//
//        }
//        else if(a.getTag()> tag)
//        {
//            m.rotate(-90);
//            m.forward(10);
//            m.rotate(-90);
//            findTag(tag);
//
//        }
//    }
    // Drive up close to tag
//    public void driveToTag()
//    {
//        double x = a.getX();
//        double y = a.getY();
//        double angle =  Math.tan(x/y);
//        int length = (int)Math.sqrt(Math.pow(x,2)+Math.pow(y,2)); // set distance to travel as the hypotenuse of a triangle with x and y as sides
//        m.rotate((int) angle);
//        m.forward(length);
//        m.rotate(-90+(int)angle);
//    }
}
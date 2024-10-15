package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name=" FarAscend")
public class AutoTestFarASCEND extends LinearOpMode {

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

        m.forward(90);
        sleep(3000);

        m.rotate(80);
        sleep(3000);

        m.forward(78);
        sleep(3000);



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
        if(opModeIsActive())
        {
            m.right(24);// Drive up to view front spike mark

            sleep(scanTimer);
            telemetry.update();
            /// sleep(3000);
            telemetry.update();

/*Pixel is straight ahead:
    0. Set tag to 2, tag will later be set to 5 if it is a red OpMode
    1. Push Pixel up to Spike Mark
    2. Move back
    3. Face Backboard
 */
            if(2==2)
            {
                tag = 2;
                telemetry.update();

            }
/* Pixel is not straight ahead:
    1. Drive up to spike marks
    2. Check if Pixel is Left
*/
            else            {
                telemetry.addData("NOT ", "Straight");
                telemetry.update();
                m.right(12);
                sleep(sleepTimer);
                telemetry.update();
                telemetry.update();
                sleep(sleepTimer);
/* Pixel is Left
    0. Set Tag to 1
    1. Rotate to face spike mark
    2. Push pixel to spike mark
    3. Move Back
    4. Face Backboard
 */
                if(1==1)
                {
                    tag = 1;
                    telemetry.addData("drive", " left");
                    telemetry.update();
                    m.forward(12);
                    // drop pixel
                    m.forward(-12);

                }
/* By process of elimination, the team prop is right
    0. Set tag to 3
    1. Rotate to face spike mark
    2. Push pixel
    3. Move back
    4. Face Backboard

 */
                else
                {
                    tag = 3;
                    telemetry.addData("drive", " right");
                    telemetry.update();
                    m.forward(-12);
                    // drop pixel
                    m.forward(12);
                    m.right(-24);

                }

            }
            telemetry.addData("driving", " to face backboard");
            telemetry.update();
            m.forward(-36); // Drive up to backboard



            //telemetry.update();
            telemetry.update();
            sleep(3000);
//           // findTag(a.getTag());
//            m.placePixel();
//
//            as.armUp();
//            as.trap();
//            as.armDown();
        }

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
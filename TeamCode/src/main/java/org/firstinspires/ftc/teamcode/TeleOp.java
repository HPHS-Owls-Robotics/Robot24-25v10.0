
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Driving", group="12417")

public class TeleOp extends LinearOpMode {

    //declaration
    //DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;
    DcMotor Slide;

    DcMotor oneHang;
    DcMotor twoHang;

    CRServo slide;
    CRServo sweep;
    Servo specimen;


    int FREEZE_SPEED=0;
//    private float cfl = 2.7f;
//    private float cfr = 1.5f;
//    private float cbl = 3.0f;
//    private float cbr = 3.0f;
    private float cfl = 0.4f;
    private float cfr = 0.4f; //was 0.8f
    private float cbl = 0.4f;
    private float cbr = 0.4f;
    private int lockPosition = 0;
    @Override

    public void runOpMode() throws InterruptedException {
        //initialization
        FRMotor = hardwareMap.dcMotor.get("FR_Motor");
        FLMotor = hardwareMap.dcMotor.get("FL_Motor");
        BLMotor = hardwareMap.dcMotor.get("BL_Motor");
        BRMotor = hardwareMap.dcMotor.get("BR_Motor");
        oneHang = hardwareMap.dcMotor.get("H1");
        twoHang = hardwareMap.dcMotor.get("H2");

        Slide = hardwareMap.dcMotor.get("S_Motor");
        slide = hardwareMap.crservo.get("slide");
        sweep = hardwareMap.crservo.get("sweep");
        specimen = hardwareMap.servo.get("specimen");


//speeds
        float SPwr=1.5f, APwr, FLPwr,FRPwr,BLPwr,BRPwr,BPwr=10f;

//booleans
        boolean isBeamBroke;
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        oneHang.setDirection(DcMotor.Direction.REVERSE);
        twoHang.setDirection(DcMotor.Direction.FORWARD);



//let's gooo
//
// ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
        waitForStart();
//        Slide.setPower(-0.5f);
//        sleep(2000);
//        Slide.setPower(0);

        while (opModeIsActive()) {
            //slide.setPower(10);
            DcMotor[] motors = {FRMotor, FLMotor, BLMotor, BRMotor/*,Slide*/, oneHang, twoHang};
            for (int i = 0; i < 6; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               //   motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

//            sweepRight.setDirection(DcMotor.Direction.FORWARD);
//            sweepLeft.setDirection(DcMotor.Direction.FORWARD);

            float yleft, yright,strafe,rotate;

            yleft = gamepad1.left_stick_y;
            yright = gamepad1.right_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;


            FLPwr =cfl*(yleft + yright - strafe - rotate); //cfl for possible coeffs to deal with weight distr
            FRPwr =cfr* (yleft + yright + strafe + rotate); //cfr for possible coeffs to deal with weight distr
            BLPwr =cbl* (yleft + yright + strafe - rotate); //cbl for possible coeffs to deal with weight distr
            BRPwr = cbr* (yleft + yright - strafe + rotate); //cbr for possible coeffs to deal with weight distr
//            FLPwr= cfl*(yright-strafe-rotate);
//            FRPwr= cfr*(yright+strafe+rotate);
//            BLPwr= cbl*(yright+strafe-rotate);
//            BRPwr= cbr*(yright-strafe);
//DRIVETRAIN*************************************************************************************************************************************
            FLMotor.setPower(FLPwr);
            FRMotor.setPower(FRPwr);
            BLMotor.setPower(BLPwr);
            BRMotor.setPower(BRPwr);

//            while(gamepad1.dpad_left)
//            {
//                FLMotor.setPower(-1);
//                BLMotor.setPower(1);
//                FRMotor.setPower(1);
//                BRMotor.setPower(-1);
//            }
//            while(gamepad1.dpad_right)
//            {
//                FLMotor.setPower(1);
//                BLMotor.setPower(-1);
//                FRMotor.setPower(-1);
//                BRMotor.setPower(1);
//            }


//SLIDE**********************************************************************************************************************************
//            if(gamepad1.left_trigger>0.1)
//            {
//                Slide.setTargetPosition(-700);
//                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Slide.setPower(0.5);
//                telemetry.addData("NO", Slide.getCurrentPosition());
//                telemetry.update();
//
//            }
            if(gamepad1.left_trigger>0.1)// floor
            {
                specimen.setPosition(1);
                Slide.setTargetPosition(1050);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(1.0);
                telemetry.addData("NO", Slide.getCurrentPosition());
                telemetry.update();
            }



//            if(gamepad1.left_bumper)
//            {
//                Slide.setTargetPosition(-1500);
//                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Slide.setPower(-0.5);
//                telemetry.addData("NO", Slide.getCurrentPosition());
//                telemetry.update();
//                //Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//               // Slide.setPower(0);
//
//            }
            if(gamepad1.right_trigger>0.1) //basket
            {
                specimen.setPosition(0);
                Slide.setTargetPosition(-4500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slide.setPower(-1.0);

                //Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // Slide.setPower(0);

            }
            if(gamepad1.dpad_down)
            {                oneHang.setPower(0.5);
            }
            while (gamepad1.dpad_up/*&&oneHang.getCurrentPosition()>lockPosition*/)
            {
                oneHang.setPower(0.5);
                twoHang.setPower(0.5);
            }
            //HORIZONTAL SLIDE***************************************************************************************************************
            while(gamepad1.y)
            {
                slide.setPower(1);
            }
            while(gamepad1.a)
            {
                slide.setPower(-1);
            }
            if(gamepad1.dpad_left)
            {
                specimen.setPosition(0.14);
            }
            if(gamepad1.dpad_right)
            {
                specimen.setPosition(0.52);
            }

            //SWEEPER**************************************************************************************************************************
            while(gamepad1.b)
            {
                sweep.setPower(0.5);
                telemetry.addData("sample", "dropped");
                telemetry.update();
            }
            while(gamepad1.x)
            {
                sweep.setPower(-0.5);
                telemetry.addData("sample", "picked");
                telemetry.update();
            }
            while(gamepad1.right_bumper&&Slide.getCurrentPosition()>-46)//&&Slide.getCurrentPosition()>-4800
            {
                Slide.setTargetPosition(Slide.getCurrentPosition()-100);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(-1.0);
                telemetry.addData("NO", Slide.getCurrentPosition());
                telemetry.update();
            }
            while(gamepad1.left_bumper)
            {
                Slide.setTargetPosition(Slide.getCurrentPosition()+100);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(1.0);
                telemetry.addData("NO", Slide.getCurrentPosition());
                telemetry.update();
            }
//            Slide.setTargetPosition(Slide.getCurrentPosition());
//            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Slide.setPower(-0.9);
//            FLMotor.setPower(0);
//            BLMotor.setPower(0);
//            FRMotor.setPower(0);
//            BRMotor.setPower(0);

           // Slide.setPower(0);
            if(!gamepad1.left_bumper&&!gamepad1.right_bumper)
            {
                slide.setPower(0);
                sweep.setPower(0);

            }
            if(!gamepad1.dpad_up)
            {
                oneHang.setPower(0);
                twoHang.setPower(0);
            }

            //slide.setPower(0);
            //sweep.setPower(0   );
            telemetry.addData("one- hang", oneHang.getCurrentPosition());
            telemetry.addData("two- hang", twoHang.getCurrentPosition());
//
//            telemetry.addData("BR", BRMotor.getPower());
//            telemetry.addData("BL", BLMotor.getPower());
//            telemetry.addData("FR", FRMotor.getPower());
//            telemetry.addData("FL", FLMotor.getPower());
            telemetry.addData("Slide", Slide.getCurrentPosition());
            telemetry.update();
        }

    }

}
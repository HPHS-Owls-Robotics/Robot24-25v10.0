package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MoveSys {


    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor Arm;
    private Servo trapdoor;
    BHI260IMU imu;



    //numbers
    float armSpeed = 0.4f;
    static final float     COUNTS_PER_MOTOR_REV    = 300f;
    static final float     DRIVE_GEAR_REDUCTION    = 1.0f;
    static final float     WHEEL_DIAMETER_INCHES   = 3.54f;
    static final float     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415f);
    static final float     DRIVE_SPEED             = 0.3F;
    static final float     TURN_SPEED              = 0.9f;
    static final float     ARM_SPEED                = 0.4f;

    //placeholder
    Orientation lastAngles = new Orientation();
    float globalAngle;

    public MoveSys(HardwareMap hardwareMap) {
        //openCv = new OpticSysOpenCV(hardwareMap);
        //aprilTag = new OpticSysAprilTag(hardwareMap);
        imu =  hardwareMap.get(BHI260IMU.class, "imu");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(myIMUparameters);
//        imu.initialize(parameters);




//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");

        FLMotor = hardwareMap.dcMotor.get ("FL_Motor"); //check with driver hub
        FRMotor = hardwareMap.dcMotor.get("FR_Motor"); //check with driver hub
        BLMotor = hardwareMap.dcMotor.get ("BL_Motor"); //check with driver hub
        BRMotor = hardwareMap.dcMotor.get ("BR_Motor"); //check with driver hub


        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    public int getCurrentTicks()
    {
        return FLMotor.getCurrentPosition();
    }
    public int forward(float inches) {
//        FRMotor.setPower(1);
//        FLMotor.setPower(1);
//        BRMotor.setPower(1);
//        BLMotor.setPower(1);
        int i = (int) (inches*COUNTS_PER_INCH);
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        // Determine new target position, and pass to motor controller
        newFL = FLMotor.getCurrentPosition() + i;
        newFR = FRMotor.getCurrentPosition() + i;
        newBL = BLMotor.getCurrentPosition() + i;
        newBR = BRMotor.getCurrentPosition() + i;

        FLMotor.setTargetPosition(newFL);
//        FRMotor.setTargetPosition(newFR);
//        BLMotor.setTargetPosition(newBL);
//        BRMotor.setTargetPosition(newBR);
//
//        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(FLMotor.getCurrentPosition()!=newFL)
        {
            FLMotor.setPower(DRIVE_SPEED);
            FRMotor.setPower(DRIVE_SPEED); //too fast
            BLMotor.setPower(DRIVE_SPEED); //wrong direction
            BRMotor.setPower(DRIVE_SPEED);
        }
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);



  return 0;
    }
    public int right(float inches) {
        int i = (int) (inches*COUNTS_PER_INCH); //convert inches to ticks
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        // Determine new target position, and pass to motor controller
        newFL = FLMotor.getCurrentPosition() - i;
        newFR = FRMotor.getCurrentPosition() + i;
        newBL = BLMotor.getCurrentPosition() + i;
        newBR = BRMotor.getCurrentPosition() - i;

        //Set values
        FLMotor.setTargetPosition(newFL);
        FRMotor.setTargetPosition(newFR);
        BLMotor.setTargetPosition(newBL);
        BRMotor.setTargetPosition(newBR);

            FLMotor.setPower(DRIVE_SPEED);
            FRMotor.setPower(DRIVE_SPEED);
            BLMotor.setPower(DRIVE_SPEED);
            BRMotor.setPower(DRIVE_SPEED);
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);



        return newFL;
    }
    public void right_old(int inches)
    {
        int newFL;
        int newBR;
        int newFR;
        int newBL;


        // Determine new target position, and pass to motor controller
        newFL = FLMotor.getCurrentPosition() -(int)(inches * COUNTS_PER_INCH);
        newBR = BRMotor.getCurrentPosition() -(int)(inches * COUNTS_PER_INCH);

        newFR = FRMotor.getCurrentPosition() +(int)(inches * COUNTS_PER_INCH);
        newBL = BLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        FLMotor.setTargetPosition(newFL);
        BRMotor.setTargetPosition(newBR);

        FRMotor.setTargetPosition(newFR);
        BLMotor.setTargetPosition(newBL);



        // Turn On RUN_TO_POSITION
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // reset the timeout time and start motion.
        //runtime.reset();
        FLMotor.setPower(DRIVE_SPEED);
        FRMotor.setPower(DRIVE_SPEED);
        BLMotor.setPower(DRIVE_SPEED);
        BRMotor.setPower(DRIVE_SPEED);

        // Turn off RUN_TO_POSITION
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public float getAngle()
    {
        Orientation myRobotOrientation;
        myRobotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        );
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
//
//        Orientation angles = imu.getAngularOrientation();

        float deltaAngle = myRobotOrientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = myRobotOrientation;

        return globalAngle;
    }

    //The method turns the robot by a specific angle, -180 to +180.
    public void rotate(int degrees)
    {

        float  leftPower, rightPower;

        resetAngle();

        //if the degrees are less than 0, the robot will turn right
        if (degrees < 0)
        {
            leftPower = TURN_SPEED;
            rightPower = -TURN_SPEED;
        }
        else if (degrees > 0)//if greater than 0, turn left
        {
            leftPower = -TURN_SPEED;
            rightPower = TURN_SPEED;
        }
        else return;

        //sets power to motors with negative signs properly assigned to make the robot go in the correct direction
        FLMotor.setPower(leftPower);
        FRMotor.setPower(rightPower);
        BLMotor.setPower(leftPower);
        BRMotor.setPower(rightPower);

        //Repeatedly check the IMU until the getAngle() function returns the value specified.
        if (degrees < 0)
        {

            while ( getAngle() > degrees) {}
        }
        else
            while (getAngle() < degrees) {}


        //stop the motors after the angle has been found.

        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);

        //sleep for a bit to make sure the robot doesn't over sh

        resetAngle();
    }


    //this method resets the angle so that the robot's heading is now 0

    public void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        );

        globalAngle = 0;
    }
    //Go to correct tag

    public void placePixel()
    {
        int target = 1500;
        Arm.setTargetPosition(1500);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(Math.abs(armSpeed));
        trapdoor.setPosition(1.0);

    }
    public void returnTrapdoor()
    {
        trapdoor.setPosition(0.0);
    }
    public void getCalibrationStatus()
    {
        return;
    }

    public boolean isMoving()
    {
        return FLMotor.getPowerFloat()&& FRMotor.getPowerFloat();

    }
}
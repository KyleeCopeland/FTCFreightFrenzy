package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@Autonomous(name="BlueCarousel")
public class BlueCarousel extends LinearOpMode{

    MainBase base = new MainBase();

    @Override
    public void runOpMode() throws InterruptedException {

        ContourDetector detector = new ContourDetector(hardwareMap, telemetry);

        base.init(hardwareMap);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        /* int position = detector.getDecision();
        detector.setTelemShow(false);

        //---------------- CASE LEFT ----------------
        if (position == 0){
            //Robot movements for "Case Left" position of team marker.
        }

        //---------------- CASE MIDDLE ----------------
        else if(position == 1){
            //Robot movements for "Case Middle" position of team marker.

        }

        //---------------- CASE RIGHT ----------------
        else{
            //Robot movements for "Case Right" position of team marker.
        }*/


        //---------------- CASE LEFT ----------------

        base.gyroDrive(-1, 13,13,0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 115, this); //Moves robot towards carousel and turns it

        base.rightDuck.setPower(0.5);
        sleep(3000);
        base.rightDuck.setPower(0); //spins carousel and delivers duck

        base.gyroTurn(base.DRIVE_SPEED, -90, this);
        base.gyroDrive(base.DRIVE_SPEED, 24, 24, 0, 0, 0, this); //moves toward alliance hub

        base.lift(1,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); // lines 66-76 -- sets down bucket, releases freight, and flips bucket up

        base.gyroDrive(-1, 6, 6, 0, 0, 0, this);
        base.gyroTurn(base.DRIVE_SPEED, -45, this);
        base.gyroDrive(0.5, 80, 80, 0, 0, 0, this); //drives into warehouse and parks


        //---------------- CASE MIDDLE ----------------

        base.gyroDrive(-1, 13,13,0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 115, this);  //Moves robot towards carousel and turns it

        base.rightDuck.setPower(0.5);
        sleep(3000);
        base.rightDuck.setPower(0); //spins carousel and delivers duck

        base.gyroTurn(base.DRIVE_SPEED, -90, this);
        base.gyroDrive(base.DRIVE_SPEED, 24, 24, 0, 0, 0, this); //moves toward alliance hub

        base.lift(2,this); //lifts bucket to second level

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); // lines 97-107 -- sets down bucket, releases freight, and flips bucket up

        base.lift(1, this);

        base.gyroDrive(-1, 6, 6, 0, 0, 0, this);
        base.gyroTurn(base.DRIVE_SPEED, -45, this);
        base.gyroDrive(0.5, 80, 80, 0, 0, 0, this); //drives into warehouse and parks


        //---------------- CASE RIGHT ----------------

        base.gyroDrive(-1, 13,13,0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 115, this);  //Moves robot towards carousel and turns it

        base.rightDuck.setPower(0.5);
        sleep(3000);
        base.rightDuck.setPower(0); //spins carousel

        base.gyroTurn(base.DRIVE_SPEED, -90, this);
        base.gyroDrive(base.DRIVE_SPEED, 24, 24, 0, 0, 0, this); //moves toward alliance hub

        base.lift(3,this); //lifts bucket to third level

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); // delivers pre-loaded game piece

        base.lift(1, this);

        base.gyroDrive(-1, 6, 6, 0, 0, 0, this);
        base.gyroTurn(base.DRIVE_SPEED, -45, this);
        base.gyroDrive(0.5, 80, 80, 0, 0, 0, this); //drives into warehouse and parks

    }
}

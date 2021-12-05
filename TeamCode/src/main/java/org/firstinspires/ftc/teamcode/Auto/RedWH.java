package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@Autonomous(name="RedWH")
public class RedWH extends LinearOpMode{

    MainBase base = new MainBase();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);


        base.init(hardwareMap);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        int position = detector.getDecision();
        detector.setTelemShow(false);
        /*
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
        */
        //---------------- CASE LEFT -----------------

        base.gyroTurn(base.DRIVE_SPEED, -90, this);
        base.gyroDrive(base.DRIVE_SPEED, 32, 32,0,0,0, this);
        base.gyroTurn(base.DRIVE_SPEED, -90, this); //turns and drives toward alliance hub

        base.lift(1,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); //delivers pre-loaded game piece

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this); //turns and drives toward warehouse

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(-0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); //collects freight

        base.gyroDrive(-0.5, 24, 24, 0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 180, this); //turns and drives toward hub

        base.lift(3,this); //lifts bucket

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); //delivers freight

        base.lift(1, this);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this); //turns and parks in warehouse


        //---------------- CASE MIDDLE ----------------

        base.gyroTurn(base.DRIVE_SPEED, -90, this);
        base.gyroDrive(base.DRIVE_SPEED, 32, 32,0,0,0, this);
        base.gyroTurn(base.DRIVE_SPEED, -90, this); //turns and drives toward hub

        base.lift(2,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); //delivers pre-loaded game piece

        base.lift(1, this);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this); //turns and drives toward warehouse

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.rightClaw.setPower(-0.5);
        sleep(500);
        base.rightClaw.setPower(0); //collects freight

        base.gyroDrive(-0.5, 24, 24, 0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 180, this); //drives and turns toward hub

        base.lift(3,this); //lifts bucket

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); //delivers freight

        base.lift(1, this);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this); //turns and parks in warehouse


        //---------------- CASE RIGHT ----------------

        base.gyroTurn(base.DRIVE_SPEED, -90, this);
        base.gyroDrive(base.DRIVE_SPEED, 32, 32,0,0,0, this);
        base.gyroTurn(base.DRIVE_SPEED, -90, this); //turns and drives toward alliance hub

        base.lift(3,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); //delivers pre-loaded game piece

        base.lift(1, this);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this); //turns and drives toward warehouse

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.rightClaw.setPower(-0.5);
        sleep(500);
        base.rightClaw.setPower(0); //collects freight

        base.gyroDrive(-0.5, 24, 24, 0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 180, this); //turns and drives toward hub

        base.lift(3,this); //lifts bucket

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0); //delivers freight

        base.lift(1, this);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this); //turns and parks in warehouse
    }
    }


package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@Autonomous(name="Main BLUE")
public class BlueWH extends LinearOpMode{

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

        /*int position = detector.getDecision();
        detector.setTelemShow(false);

        //---------------- CASE LEFT ----------------
        if (position == 0){
            //Robot movements for "Case Left" position of team marker.
        robot.gyroDrive(MainBase.
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

        base.gyroDrive(base.DRIVE_SPEED, 32, 32,0,0,0, this);
        base.gyroTurn(base.DRIVE_SPEED, 90, this);

        base.lift(1,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.rightClaw.setPower(-0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.gyroDrive(-0.5, 24, 24, 0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 180, this);

        base.lift(3,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this);


        //---------------- CASE MIDDLE ----------------

        base.gyroDrive(base.DRIVE_SPEED, 32, 32,0,0,0, this);
        base.gyroTurn(base.DRIVE_SPEED, 90, this);

        base.lift(2,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.rightClaw.setPower(-0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.gyroDrive(-0.5, 24, 24, 0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 180, this);

        base.lift(3,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this);


        //---------------- CASE RIGHT ----------------

        base.gyroDrive(base.DRIVE_SPEED, 32, 32,0,0,0, this);
        base.gyroTurn(base.DRIVE_SPEED, 90, this);

        base.lift(3,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.rightClaw.setPower(-0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.gyroDrive(-0.5, 24, 24, 0,0,0,this);
        base.gyroTurn(base.DRIVE_SPEED, 180, this);

        base.lift(3,this);

        base.bucket.setPower(0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.rightClaw.setPower(0.5);
        sleep(500);
        base.rightClaw.setPower(0);

        base.bucket.setPower(-0.5);
        sleep(2000);
        base.bucket.setPower(0);

        base.gyroTurn(base.DRIVE_SPEED, 180, this);
        base.gyroDrive(0.5, 24, 24, 0,0,0,this);

    }
}

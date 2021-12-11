package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.MainBase;

@TeleOp(name="MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    MainBase base = null;

    public boolean GP1_RB_Held   = false;
    public boolean GP2_LB_Held   = false;
    public boolean GP2_RB_Held   = false;
    public boolean GP2_Y_Held    = false;
    public boolean GP2_A_Held    = false;
    public boolean SlowMode      = false;
    public boolean AUTO_LIFT     = false;
    public double  LCLAW_OPEN    = 0.75; //0.75;
    public double  LCLAW_CLOSED  = 0.25; //Delux hitec 485HB
    public double  RCLAW_OPEN    = 0.25;
    public double  RCLAW_CLOSED  = 0.75; //0.6;
    public double  BUCKET_OPEN   = 0.75; //0.5;
    public double  BUCKET_HALF   = 0.5; //posistion halfway for delivering
    public double  BUCKET_CLOSED = 0.25; //0.3;
    public double  DUCK_SPEED = -0.42;
    int level = 0;


    @Override
    public void runOpMode() {
        custom_init();
        waitForStart();
        while (opModeIsActive()) {
            custom_loop();
        }
    }

    public void custom_init() {
        base = new MainBase();
        base.init(hardwareMap);

        telemetry.addData("Initialization Complete!", "");
        telemetry.update();
    }

    public void custom_loop() {

        //--------------------DRIVE-TRAIN CONTROLS--------------------\\
        double forward = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower = forward + turn;
        double rightPower = forward - turn;
        double[] powers = {leftPower, rightPower};

        boolean needToScale = false;
        for (double power : powers) {
            if (Math.abs(power) > 1) {
                needToScale = true;
                break;
            }
        }
        if (needToScale) {
            double greatest = 0;
            for (double power : powers) {
                if (Math.abs(power) > greatest) {
                    greatest = Math.abs(power);
                }
            }
            leftPower /= greatest;
            rightPower /= greatest;
        }

        //--------------------SLOW-MODE--------------------\\
        if (gamepad1.right_bumper && !GP1_RB_Held) {
            GP1_RB_Held = true;
            SlowMode = !SlowMode;
        }
        if (!gamepad1.right_bumper) {
            GP1_RB_Held = false;
        }

        if (SlowMode) {
            base.leftDT.setPower(0.3 * leftPower);
            base.rightDT.setPower(0.3 * rightPower);
        } else {
            base.leftDT.setPower(leftPower);
            base.rightDT.setPower(rightPower);
        }


        //--------------------ROBOT CONTROLS--------------------\\

        //---------------LEFT-DUCK---------------\\
        if (gamepad2.x) {
            base.leftDuck.setPower(-DUCK_SPEED);
        } else {
            base.leftDuck.setPower(0);
        }

        //---------------RIGHT-DUCK---------------\\
        if (gamepad2.b) {
            base.rightDuck.setPower(-DUCK_SPEED);
        } else {
            base.rightDuck.setPower(0);
        }

        //---------------LIFT-SYSTEM---------------\\
        if (gamepad2.a && !GP2_A_Held) {
            GP2_A_Held = true;
            base.bucket.setPosition(BUCKET_HALF);
        } else {
            base.bucket.setPosition(0);
        }

        double liftArm = -gamepad2.right_stick_y;
        if (Math.abs(liftArm) < 0.1) {
            if (gamepad2.dpad_up) {
                AUTO_LIFT = true;
                level = 3;
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                AUTO_LIFT = true;
                level = 2;
            } else if (gamepad2.dpad_down) {
                AUTO_LIFT = true;
                level = 1;
            }
            if (AUTO_LIFT) {
                base.lift(level, this);
            } else {
                base.lift.setPower(0);
            }
        } else {
            base.lift.setPower(liftArm);
            AUTO_LIFT = false;
        }

        //---------------LEFT-CLAW---------------\\
        if (gamepad2.left_bumper && !GP2_LB_Held) {
            GP2_LB_Held = true;
            if (base.leftClaw.getPosition() == LCLAW_CLOSED) {
                base.leftClaw.setPosition(LCLAW_OPEN);
            } else {
                base.leftClaw.setPosition(LCLAW_CLOSED);
            }
        }
        if (!gamepad2.left_bumper) {
            GP2_LB_Held = false;
        }

        //---------------RIGHT CLAW---------------\\
        if (gamepad2.right_bumper && !GP2_RB_Held) {
            GP2_RB_Held = true;
            if (base.rightClaw.getPosition() == RCLAW_CLOSED) {
                base.rightClaw.setPosition(RCLAW_OPEN);

                telemetry.addData("RIGHT CLAW OPEN","");
            } else {
                base.rightClaw.setPosition(RCLAW_CLOSED);

                telemetry.addData("RIGHT CLAW CLOSED","");
            }
        }
        //telemetry.update();
        if (!gamepad2.right_bumper) {
            GP2_RB_Held = false;
        }


        //---------------BUCKET---------------\\
        if (gamepad2.y && !GP2_Y_Held) {
            GP2_Y_Held = true;
            if (base.bucket.getPosition() == BUCKET_CLOSED) {
                base.bucket.setPosition(BUCKET_OPEN);

                telemetry.addData("BUCKET OPEN","");
            } else {
                base.bucket.setPosition(BUCKET_CLOSED);

                telemetry.addData("BUCKET CLOSED","");
            }
            telemetry.update();
        }
        if (!gamepad2.y) {
            GP2_Y_Held = false;
        }

    }
}
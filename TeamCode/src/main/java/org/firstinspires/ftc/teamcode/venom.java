package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "venom")
public class venom extends OpMode {
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;
    DcMotor gearTrain;
    Servo hook;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        gearTrain = hardwareMap.dcMotor.get("gearTrain");

        hook = hardwareMap.servo.get("hook");



        /*fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/
    }

    @Override
    public void loop() {

        //Front back Left
        if (Math.abs(gamepad1.left_stick_y) > .2) {
            fl.setPower(gamepad1.left_stick_y * 1);
            bl.setPower(gamepad1.left_stick_y * -1);
        } else {
            fl.setPower(0);
            bl.setPower(0);
        }

        //Front back Right
        if (Math.abs(gamepad1.right_stick_y) > .2) {
            fr.setPower(gamepad1.right_stick_y * -1);
            br.setPower(gamepad1.right_stick_y * 1);
        } else {
            fr.setPower(0);
            br.setPower(0);
        }

        //Side speed Right
        if (gamepad1.right_bumper) {
            fl.setPower(1);
            bl.setPower(1);
            fr.setPower(1);
            br.setPower(1);
        } else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }

        //Side speed Left
        if (gamepad1.left_bumper) {
            fl.setPower(-1);
            bl.setPower(-1);
            fr.setPower(-1);
            br.setPower(-1);
        } else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }

        // Hang to go up and down
        if (gamepad2.dpad_down) {
            gearTrain.setPower(.9);

        } else if (gamepad2.dpad_down) {
            gearTrain.setPower(-.9);

        } else {
            gearTrain.setPower(0);
        }
        //This is the claw have to tweak it
        if (gamepad2.dpad_right) {
            hook.setPosition(1);

        } else if (gamepad2.dpad_left) {
            hook.setPosition(0.20);
        }
    }
}

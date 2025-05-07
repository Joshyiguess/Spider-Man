package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "venom")
public class venom extends OpMode {
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;
    DcMotor leftHang;
    DcMotor rightHang;
    Servo claw;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        leftHang = hardwareMap.dcMotor.get("leftHang");
        rightHang = hardwareMap.dcMotor.get("rightHang");

        claw = hardwareMap.servo.get("claw");



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
            leftHang.setPower(.9);
            rightHang.setPower(.9);

        } else if (gamepad2.dpad_down) {
            leftHang.setPower(-.9);
            rightHang.setPower(-.9);

        } else {
            leftHang.setPower(0);
            rightHang.setPower(0);
        }
        //This is the claw have to tweak it
        if (gamepad2.dpad_right) {
            claw.setPosition(1);

        } else if (gamepad2.dpad_left) {
            claw.setPosition(0.20);
        }
    }
}

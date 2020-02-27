package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Driver Controlled Op Mode")
public class DriverControlledOpMode extends OpMode {

    DcMotor left_front;
    DcMotor left_back;
    DcMotor right_front;
    DcMotor right_back;
    double G1rightStickY = 0;
    double G1leftStickY = 0;


    @Override
    public void init() {



        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        G1rightStickY = -gamepad1.right_stick_y;
        G1leftStickY = -gamepad1.left_stick_y;
        right_front.setPower(-G1rightStickY);
        right_back.setPower(-G1rightStickY);
        left_front.setPower(G1leftStickY);
        left_back.setPower(G1leftStickY);

    }

    @Override
    public void stop(){

    }
}
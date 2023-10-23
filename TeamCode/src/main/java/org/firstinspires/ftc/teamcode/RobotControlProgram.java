
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
@Disabled

public class RobotControlProgram extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor testMotor1;
    private DcMotor testMotor2;
    private DcMotor armMotor1;
    private DcMotor armMotor2;
    private Servo armservo;
    private Gyroscope imu;
        static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    double position = (MAX_POS + MIN_POS) / 2;

       @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        testMotor1 = hardwareMap.get(DcMotor.class, "TestMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "TestMotor2");
        armMotor1 = hardwareMap.get(DcMotor.class, "TestMotor3");
        armMotor2 = hardwareMap.get(DcMotor.class, "TestMotor4");
        armservo = hardwareMap.get(Servo.class, "armservo");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        //initialize the first thicc motor
        testMotor1.setPower(0);
        testMotor2.setPower(0);
        armMotor1.setPower(0);
        armMotor2.setPower(0);
        armservo.setPosition(1);


        //motor mode time lolll
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armservo.setPosition(position);
        //stop while no input
        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftStick;
            double rightStick;
            double rightTrigger;
            double leftTrigger;
            double lMotorPwr;
            double rMotorPwr;
            double arm1power;
            double arm2power;
            double leftStick2;
            double rightStick2;
            double rightTrigger2;
            double leftTrigger2;
        
            leftStick = gamepad1.left_stick_x;
            rightStick = gamepad1.right_stick_x;
            rightTrigger = gamepad1.right_trigger;
            leftTrigger = gamepad1.left_trigger;
             leftStick2 = gamepad2.left_stick_y;
            rightStick2 = gamepad2.right_stick_y;
            rightTrigger2 = gamepad2.right_trigger;
            leftTrigger2 = gamepad2.left_trigger;
            
            lMotorPwr = rightTrigger - leftTrigger  - leftStick;
            rMotorPwr = rightTrigger - leftTrigger + leftStick;
            arm1power = leftStick2;
            telemetry.addData("lmotor power", "%.3f", lMotorPwr);
            telemetry.addData("rmotor power", "%.3f", rMotorPwr);
            telemetry.addData("arm1power", "%.3f", arm1power);
            
            telemetry.update();
            testMotor1.setPower(lMotorPwr);
            testMotor2.setPower(-rMotorPwr);
            armMotor1.setPower(0.5 * -arm1power);
            armMotor2.setPower(0.5 * arm1power);
             if (gamepad2.a) {
                armservo.setPosition(0);
            }
            if (gamepad2.b) {
                armservo.setPosition(1);
            }
            
        }
    }
}

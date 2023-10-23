package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (group = "2023")

public class ArmServoTestOpMode extends LinearOpMode {
    private Blinker control_Hub;
    private Servo bottomArmServo;
    private Gyroscope imu;
    private Servo topArmServo;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null; //front left
    private DcMotor motor2 = null; //front right
    private DcMotor motor3 = null; //back left
    private DcMotor motor4 = null; //back right
    private DcMotor armmotor = null;

    @Override
    public void runOpMode() {

        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        bottomArmServo = hardwareMap.get(Servo.class, "bottomArmServo");
        topArmServo = hardwareMap.get(Servo.class, "topArmServo");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        armmotor = hardwareMap.get(DcMotor.class, "armcontrol");
        
    
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        double lTrigger;
        double rTrigger;
        double lStickX;
        double lStickY;
        double rStickX;
        boolean lBumper;
        boolean rBumper;
        boolean bottomArmServoStatus = false;
        boolean topArmServoStatus = false;
        boolean lBumperDown = false;
        boolean rBumperDown = false;
        
        double slow_mode = 0.25;
        telemetry.addData("From:", "Android Studio");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            rTrigger = gamepad1.right_trigger;
            lTrigger = gamepad1.left_trigger;
            lBumper = gamepad1.left_bumper;
            rBumper = gamepad1.right_bumper;
            
            if (lBumper) {
                if (!lBumperDown) {
                    if (bottomArmServoStatus) {
                        bottomArmServoStatus = false;
                    }
                    else {
                        bottomArmServoStatus = true;
                    }
                    lBumperDown = true;
                }
            }
            else {
                lBumperDown = false;
            }
            if (rBumper) {
                if (!rBumperDown) {
                    if (topArmServoStatus) {
                        topArmServoStatus = false;
                    }
                    else {
                        topArmServoStatus = true;
                    }
                    rBumperDown = true;
                }
            }
            else {
                rBumperDown = false;
            }
            
            if (bottomArmServoStatus) {
                bottomArmServo.setPosition(0);
            }
            else {
                bottomArmServo.setPosition(0.15);
            }
            if (topArmServoStatus) {
                topArmServo.setPosition(0.6);
            }
            else {
                topArmServo.setPosition(0.45);
            }
            
            armmotor.setPower(lTrigger*lTrigger);
            armmotor.setPower(-rTrigger*rTrigger);
            
            //bottomArmServo.setPosition((1-lTrigger)*0.15);
            //topArmServo.setPosition(rTrigger*0.15+0.45);
            
            //close:
            //bottomArmServo.setPosition(0);
            //topArmServo.setPosition(0.6);
            
            //open:
            //bottomArmServo.setPosition(0.15);
            //topArmServo.setPosition(0.45);
    
            lStickX = slow_mode*(gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x));
            lStickY = slow_mode*(-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
            rStickX = slow_mode*(gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));
    
            double distanceFrom0 = Math.sqrt(Math.pow(lStickX, 2) + Math.pow(lStickY, 2));
    
            double direction = findDir(lStickX * 1.0/distanceFrom0, lStickY * 1.0/distanceFrom0);
            
            telemetry.addData("Stick Powers", ":lStickX: %.2f, lStickY: %.2f, RStickX:%.2f", lStickX, lStickY, rStickX);
    
            setMotorPowers(direction, distanceFrom0, rStickX);
            
            telemetry.addData("Status", "Running");
            //telemetry.addData("Servos", "Bottom: %.3f, Top: %.3f", lTrigger, rTrigger);
            telemetry.update();

        }
    }
    public double findDir(double x, double y) {
        double temp = Math.toDegrees(Math.asin(y));
        if (y >= 0) {
            if (x >= 0) {
                return temp;
            }
            else {
                return 180 - temp;
            }
        }
        else {
            if (x >= 0) {
                return temp;
            }
            else {
                return -180 - temp;
            }
        }
    }
    public void setMotorPowers(double dir, double dist, double turn) {
        
        double motor1Power = 0;
        double motor2Power = 0;
        double motor3Power = 0;
        double motor4Power = 0;
        double temp;
        if (dir <= 90 && dir > 0) {
            motor1Power = dist;
            motor4Power = dist;
            temp = dir/45.0 - 1;
            motor2Power = temp * dist;
            motor3Power = temp * dist;
        }
        else if (dir <= 0 && dir > -90) {
            temp = (dir + 90)/45.0 - 1;
            motor1Power = temp * dist;
            motor4Power = temp * dist;
            motor2Power = -dist;
            motor3Power = -dist;
        }
        else if (dir <= -90 && dir > -180) {
            motor1Power = -dist;
            motor4Power = -dist;
            temp = (dir + 180)/45.0 - 1;
            motor2Power = temp * -dist;
            motor3Power = temp * -dist;
        }
        else if (dir <= 180 && dir > 90) {
            motor2Power = dist;
            motor3Power = dist;
            temp = (dir - 90)/45.0 - 1;
            motor1Power = - temp * dist;
            motor4Power = - temp * dist;
        }
        
        motor1Power += turn;
        motor3Power += turn;
        
        motor2Power -= turn;
        motor4Power -= turn;
        
        motor1.setPower(motor1Power);
        motor2.setPower(-motor2Power);
        motor3.setPower(motor3Power);
        motor4.setPower(-motor4Power);
        telemetry.addData("Wheel Powers", "Wheel1: %.2f, Wheel2: %.2f, Wheel3:%.2f, Wheel 4: %.2f", motor1Power, motor2Power, motor3Power, motor4Power);
    }
}

package org.firstinspires.ftc.teamcode.subsystems.Outtake;


import static org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage.side;
import static org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants.CLOSE_VELOCITY;


import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage;
import org.firstinspires.ftc.teamcode.pid.MiniPID;


public class Outtake {
    public DcMotorEx motor1;
    public DcMotorEx motor2;
    Servo hood;
    MiniPID velocityController;
    public double pidOutput;
    double error;
    public double SETPOINT;
    MultipleTelemetry telemetry;


    public static double P = 500, I = 0, D = 0, F = 14.3;
//    public static double K = 0.0035; // saturation rate for the hood function, needs to be tuned
//    double MIN_HOOD = 20; // need to determine this, btw these are all in degrees
//    double MAX_HOOD = 60; // need to determine this


    public Outtake(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        motor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        hood = hardwareMap.get(Servo.class, "hood");


        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motor1.setDirection(DcMotorEx.Direction.REVERSE);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);


        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));


        velocityController = new MiniPID(P, I, D, F);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
    }
    public double getVelocity() {
        return motor1.getVelocity();
    }
    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }


    public void setVelocity(double velocity) {
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }


    public void autoVelocity(double Rx, double Ry) {
        // calculate hood first
//        double hoodPos = hoodCalc(Rx, Ry);  // hoodCalc just returns a value
//        hood.setPosition(hoodPos);          // then apply it to the servo
        double hoodPos = 0;


        // calculate velocity based on hood
        int velocity = veloCalc(Rx, Ry, hoodPos);  // use hoodPos as input


        // apply PID to reach velocity
        velocityController.setSetpoint(velocity);
        pidOutput = velocityController.getOutput(Math.abs(getVelocity()));
//        telemetry.addData("PID Output", pidOutput);
//        telemetry.addData("Setpoint", velocity);
//        telemetry.addData("Error", velocity - (motor1.getVelocity()+ motor2.getVelocity())/2);
        setPower(pidOutput);
    }


    public void shootVelocity(int velocity) {
       /*
       velocityController.setSetpoint(velocity);
       pidOutput = velocityController.getOutput(Math.abs(getVelocity()));
       telemetry.addData("PID Output", pidOutput);
       telemetry.addData("Setpoint", velocity);
       telemetry.addData("Error", velocity - (motor1.getVelocity()+ motor2.getVelocity())/2);
       setPower(pidOutput);
       */


        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));


        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);


    }
    public void shootStop() {
        motor1.setPower(0);
        motor2.setPower(0);
    }
    public int veloCalc(double Rx, double Ry, double hoodPos) {
        int velocity = 5500; /* formula that depends on hoodPos, Rx, Ry */
        return velocity;
    }
//    public double hoodCalc(double Rx, double Ry) {
//        double GOAL_X = 72;
//        double GOAL_Y = 72;
//        if (side.equals(PoseStorage.SIDE.RED)) {
//            GOAL_Y = -72;
//        }
//
//        double dx = GOAL_X - Rx;
//        double dy = GOAL_Y - Ry;
//        double d = Math.sqrt(dx*dx + dy*dy);
//
//        double rawHood = MIN_HOOD + (MAX_HOOD - MIN_HOOD) * (1 - Math.exp(-K * d)); // saturating exponential model, depends on K constant to determine rate of flattening out
//
//        double hood = Math.max(MIN_HOOD, Math.min(rawHood, MAX_HOOD));
//
//        return (hood - MIN_HOOD) / (MAX_HOOD - MIN_HOOD); // this gives a range between 0 and 1 for the servo, will define constraints later
//    }


    public Action stopAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                motor1.setPower(0);
                motor2.setPower(0);
                return false;
            }
        };
    }


    public Action shootVelocityTimeAction(int vel, double time) {


        return new Action() {


            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if (!init) {
                    init = true;
                }




                SETPOINT = vel;
                velocityController.setSetpoint(SETPOINT);
                error = SETPOINT - Math.abs(getVelocity());


                pidOutput = velocityController.getOutput(Math.abs(getVelocity()));
                if (timer.seconds() < time) {
                    setPower(pidOutput);
                    timer.reset();
                }
                else {
                    setPower(0);
                    timer.reset();
                }




                telemetryPacket.put("VELOCITY", Math.abs(getVelocity()));
                telemetryPacket.put("ERROR", error);
                telemetryPacket.put("SETPOINT", SETPOINT);




                return error >= 50;
            }


        };
    }
    public Action shootVelocityAction(int vel) {


        return new Action() {


            private boolean init = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if (!init) {
                    init = true;
                }




                SETPOINT = vel;
                velocityController.setSetpoint(SETPOINT);
                error = SETPOINT - Math.abs(getVelocity());


                pidOutput = velocityController.getOutput(Math.abs(getVelocity()));
                setPower(pidOutput);






                telemetryPacket.put("VELOCITY", Math.abs(getVelocity()));
                telemetryPacket.put("ERROR", error);
                telemetryPacket.put("SETPOINT", SETPOINT);




                return error >= 50;
            }


        };
    }
    public Action reverseAction(double time) {
        return new Action() {
            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    init = true;
                }
                if (timer.seconds() < time) {
                    setPower(-0.5);
                    timer.reset();
                }
                else {
                    setPower(0);
                    timer.reset();
                }
                return false;
            }
        };
    }
    public Action reverseTimeAction(double time) {
        return  new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (timer.seconds() < time) {
                    motor1.setPower(-0.5);
                    motor2.setPower(-0.5);
                }
                else {
                    motor1.setPower(0);
                    motor2.setPower(0);
                }
                return false;
            }
        };
    }




}

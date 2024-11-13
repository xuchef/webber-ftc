package ca.webber.ftc.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ca.webber.ftc.robot.roadrunner.MecanumDrive;
import ca.webber.ftc.robot.roadrunner.StandardTrackingWheelLocalizer;
import ca.webber.ftc.subsystems.Shooter;
import ca.webber.ftc.subsystems.WobbleGrabber;

public class Omnibot {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private final MecanumDrive drive;
    private final Shooter shooter;
    private final Motor conveyor, intake;
    private final Servo wobbleGrab;
    private final CRServo wobbleLift, intakeTop;
    private final TouchSensor limitTop, limitBottom;
    private final WobbleGrabber wobbleGrabber;
    private final Motor fL, fR, bL, bR;
    private StandardTrackingWheelLocalizer odometry;

    public Omnibot(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = new GamepadEx(opMode.gamepad1);
        this.gamepad2 = new GamepadEx(opMode.gamepad2);

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        drive = new MecanumDrive(hardwareMap);

        conveyor = new Motor(hardwareMap, "conveyor");
        intake = new Motor(hardwareMap, "intake");
        intakeTop = hardwareMap.get(CRServo.class, "intakeTop");

        shooter = new Shooter((DcMotorEx) hardwareMap.get(DcMotor.class, "leftShooter"),
                (DcMotorEx) hardwareMap.get(DcMotor.class, "rightShooter"),
                hardwareMap.get(Servo.class, "feeder"));

        // Wobble Grabber
        wobbleGrab = hardwareMap.get(Servo.class, "wobbleGrab");
        wobbleLift = hardwareMap.get(CRServo.class, "wobbleLift");
        limitTop = hardwareMap.touchSensor.get("limitTop");
        limitBottom = hardwareMap.touchSensor.get("limitBottom");
        wobbleGrabber = new WobbleGrabber(wobbleLift, wobbleGrab, limitTop, limitBottom);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize localizer
        odometry = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    public Omnibot(OpMode opMode, boolean x) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = new GamepadEx(opMode.gamepad1);
        this.gamepad2 = new GamepadEx(opMode.gamepad2);

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        drive = new MecanumDrive(hardwareMap);

        conveyor = new Motor(hardwareMap, "conveyor");
        intake = new Motor(hardwareMap, "intake");
        intakeTop = hardwareMap.get(CRServo.class, "intakeTop");

        shooter = new Shooter((DcMotorEx) hardwareMap.get(DcMotor.class, "leftShooter"),
                (DcMotorEx) hardwareMap.get(DcMotor.class, "rightShooter"),
                hardwareMap.get(Servo.class, "feeder"));

        // Wobble Grabber
        wobbleGrab = hardwareMap.get(Servo.class, "wobbleGrab");
        wobbleLift = hardwareMap.get(CRServo.class, "wobbleLift");
        limitTop = hardwareMap.touchSensor.get("limitTop");
        limitBottom = hardwareMap.touchSensor.get("limitBottom");
        wobbleGrabber = new WobbleGrabber(wobbleLift, wobbleGrab, limitTop, limitBottom);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void drive(Pose2d pose) {
        drive.setWeightedDrivePower(pose);
    }

    public StandardTrackingWheelLocalizer getOdometry() {
        return odometry;
    }

    public Pose2d getPose() {
        return odometry.getPoseEstimate();
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Motor getConveyor() {
        return conveyor;
    }

    public Motor getIntake() {
        return intake;
    }

    public CRServo getIntakeTop() {
        return intakeTop;
    }

    public WobbleGrabber getWobbleGrabber() {
        return wobbleGrabber;
    }
}

package org.firstinspires.ftc.teamcode.competition;

import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp(name="manual drive", group="competition")
public class ManualDrive extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();

    //control variables
    public static final double SHOOTER_INTERVAL = 0.2;

    //apriltag/camera variables
//    private static final boolean USE_WEBCAM = true;
//    private Position cameraPosition = new Position(DistanceUnit.INCH,
//            0, 0, 0, 0);
//    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//            0, -90, 0, 0);
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
//    public static final double AUTO_TURN = 0.3;

// 6 7 8 9 14
    @Override
    public void runOpMode() {

//        initAprilTag();
        //shooter variables
        boolean shooterEnabled = false;
        boolean justChangedSpeed = false;
//        double shooterPower = 0.2;
        int speedIndex = 0;
//        double lastPosition = 0.0;
        double[] speeds = {0.5, 0.65, 0.7};
        //hardware assigning, make sure device names in here match the ones in config
        //hardware variables
        DcMotorEx frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontL");
        DcMotorSimple backLeftDrive = hardwareMap.get(DcMotorSimple.class, "backL");
        DcMotorEx frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontR");
        DcMotorEx backRightDrive = hardwareMap.get(DcMotorEx.class, "backR");
        Servo intake = hardwareMap.get(Servo.class, "intake");
//        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        Servo intake = hardwareMap.get(Servo.class, "intake");

        //directions of wheels, may need to change directions to drive properly
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        //direction of shooter
//        shooter.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //joystick variables
            double max;
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            //auto aim function, points towards certain apriltags within view
//            if (gamepad1.a) {
//                List<AprilTagDetection> detect = aprilTag.getDetections();
//                for (AprilTagDetection dect : detect){
//                    if (dect.metadata != null){
//                        if (!dect.metadata.name.contains("Obelisk")){
//                            if(dect.ftcPose.x > 0){
//                                yaw = AUTO_TURN;
//                                telemetry.addData("Auto Aim", "Turning right");
//                            }else if(dect.ftcPose.x < 0){
//                                yaw = -AUTO_TURN;
//                                telemetry.addData("Auto Aim", "Turning left");
//                            }
//                        }
//                    }else{
//                        telemetry.addData("Auto Aim", "No target found");
//                    }
//                }
//            }

            //drive and turning calculations
            double frontLeftPower  = -axial - lateral - yaw;
            double frontRightPower = -axial + lateral + yaw;
            double backLeftPower   = -axial + lateral - yaw;
            double backRightPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }


//            frontLeftPower = gamepad1.a ? -1.0 : 0.0;
//            backLeftPower = gamepad1.a ? -1.0 : 0.0;
//            frontRightPower = gamepad1.a ? -1.0 : 0.0;
//            backRightPower = gamepad1.a ? 1.0 : 0.0;

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            //shooter controls, allows for precise power setting mid match
//            if (gamepad1.right_bumper && !shooterToggle && shooterPower < 1) {
//                shooterPower += SHOOTER_INTERVAL;
//                shooter.setPower(shooterPower);
//                shooterToggle = true;
//            } else if (gamepad1.left_bumper && !shooterToggle && shooterPower > -0.2) {
//                shooterPower -= SHOOTER_INTERVAL;
//                shooter.setPower(shooterPower);
//                shooterToggle = true;
//            } else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
//                shooterToggle = false;
//            }
            //shooter controls to go right to zero or max

            if (gamepad1.dpad_down && !justChangedSpeed) {
//                shooterPower -= 0.05;
//                shooterPower = Math.max(shooterPower, 0.2);
                speedIndex -= 1;
                speedIndex = Math.max(speedIndex, 0);
                justChangedSpeed = true;
            }
            if (gamepad1.dpad_up && !justChangedSpeed) {
//                shooterPower += 0.05;
//                shooterPower = Math.min(shooterPower, 1.0);
                speedIndex += 1;
                speedIndex = Math.min(speedIndex, speeds.length - 1);
                justChangedSpeed = true;
            }
            if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                justChangedSpeed = false;
            }

            if (gamepad1.left_trigger > 0.5) {
                intake.setPosition(0.0);
            } else if (gamepad1.right_trigger > 0.5) {
                intake.setPosition(1.0);
            } else {
                intake.setPosition(0.5);
            }

//            if (gamepad1.right_trigger > 0.5){
//                shooterEnabled = true;
////                shooterPower = 1;
////                shooter.setPower(shooterPower);
//            }
//            if (gamepad1.left_trigger > 0.5){
//                shooterEnabled = false;
//            }
//            if (shooterEnabled) {
////                double velocity = shooterEnc.getDeltaPosition() - lastPosition;
//                telemetry.addData("Shooter power:", "%4.2f", speeds[speedIndex]);
////                lastPosition = shooterEnc.getDeltaPosition();
//
////                shooter.setPower(speeds[speedIndex]);
//            } else {
////                shooter.setPower(0.0);
////                shooterEnc.setDirection(0.0);
//            }


            //intake controls
//            if (gamepad1.left_bumper){
//                intake.setPosition(1.0);
//            }else if (gamepad1.right_bumper){
//                intake.setPosition(0.0);
//            }else{
//                intake.setPosition(0.5);
//            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Shooter power", "%4.2f", speeds[speedIndex]);
            telemetry.update();
        }
    }
}

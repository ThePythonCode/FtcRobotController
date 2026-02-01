package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// goBUILDA Pinpoint import
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "AdvancedRobotTest_PinPoint (With Speed Control)")
public class AdvancedRobotTest_PinPoint extends LinearOpMode {

    // ====== HARDWARE DECLARATIONS ======
    private DcMotorEx backLeft, backRight, frontLeft, frontRight;
    private DcMotorEx launcher1, launcher2, intakeMotor, liftMotor;
    private Servo torqueArmServo, torque2;
    
    // ====== ODOMETRY HARDWARE ======
    private GoBildaPinpointDriver odometryComputer;
    private boolean odometryInitialized = false;

    // Servo positions (clear constants)
    private static final double SERVO_REST_POS = 0.7;
    private static final double SERVO_SHOOT_POS = -10;
    
    // ====== SPEED CONTROL ======
    private double speedMultiplier = 1.0; // Start at 100% speed
    private static final double SPEED_INCREMENT = 0.1; // 10% per button press
    private static final double MIN_SPEED = 0.2; // Minimum 20% speed
    private static final double ddD = 0.5; // Maximum 100% speed
    
    // Button debouncing for speed controls
    private boolean lastTriangle = false;
    private boolean lastCircle = false;
    private boolean lastSquare = false;

    @Override
    public void runOpMode() {

        // ====== INITIALIZE HARDWARE ======
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        torqueArmServo = hardwareMap.get(Servo.class, "torqueArmServo");
        torque2 = hardwareMap.get(Servo.class, "torque2");

        // ====== INITIALIZE ODOMETRY ======
        initializePinpoint();

        // Reverse one side of the drivetrain so forward is consistent
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to use encoders
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        torqueArmServo.setPosition(SERVO_REST_POS);

        // Stop all motors on init
        stopAllMotors();

        // Servo initial position
        torqueArmServo.setPosition(SERVO_REST_POS);
        torque2.setPosition(0.03);
        telemetry.addLine("Initialized and ready! Servo resting at " + SERVO_REST_POS);
        telemetry.addData("Drive Speed", String.format("%.0f%%", speedMultiplier * 100));
        if (odometryInitialized) {
            telemetry.addData("Pinpoint Status", odometryComputer.getDeviceStatus());
        } else {
            telemetry.addData("Pinpoint Status", "FAILED - Check Configuration!");
        }
        telemetry.addLine("\n--- SPEED CONTROLS ---");
        telemetry.addData("Triangle (Y)", "Increase Speed +10%");
        telemetry.addData("Square (X)", "Decrease Speed -10%");
        telemetry.addData("Circle (B)", "Toggle Turbo/Normal");
        telemetry.update();

        waitForStart();
        
        // ====== RESET ODOMETRY AT START ======
        if (odometryInitialized) {
            odometryComputer.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            odometryComputer.resetPosAndIMU();
        }

        // ====== MAIN LOOP ======
        while (opModeIsActive()) {

            // ====== UPDATE ODOMETRY ======
            Pose2D pos = null;
            if (odometryInitialized) {
                odometryComputer.update();
                pos = odometryComputer.getPosition();
            }

            // ====== SPEED ADJUSTMENT CONTROLS ======
            handleSpeedAdjustments();

            // ============ DRIVE CONTROL ============
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double dz = 0.05;
            if (Math.abs(y) < dz) y = 0;
            if (Math.abs(x) < dz) x = 0;
            if (Math.abs(rx) < dz) rx = 0;
            
            if (Math.abs(rx) > 0.2 && Math.hypot(x,y) < 0.1){
                x = 0;
                y = 0;
            }
            double frontLeftPower = y + x + rx;
            double backLeftPower  = y - x + rx;
            double frontRightPower= y - x - rx;
            double backRightPower = y + x - rx;

            double max = Math.max(1.0, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));

            double maxTicksPerSec = 2000.0 * speedMultiplier; // Apply speed multiplier

            frontLeft.setVelocity((frontLeftPower / max) * maxTicksPerSec);
            backLeft.setVelocity((backLeftPower / max) * maxTicksPerSec);
            frontRight.setVelocity((frontRightPower / max) * maxTicksPerSec);
            backRight.setVelocity((backRightPower / max) * maxTicksPerSec);

            // ============ INTAKE CONTROL ============
            if (gamepad2.dpad_up) {
                intakeMotor.setVelocity(-3000.0);
            } else if (gamepad2.dpad_down) {
                intakeMotor.setVelocity(3000.0);
            } else if (gamepad2.cross) {
                intakeMotor.setVelocity(0);
            }

            // ============ ELEVATOR SYSTEM ============
            liftMotor.setVelocity(gamepad2.left_trigger > 0.5 ? 3000.0 : 0);

            // ============ SHOOTER CONTROL ============
            if (gamepad2.right_trigger > 0.5) {
                launcher1.setPower(-0.45);
                launcher2.setPower(0.45);

            } else {
                launcher1.setVelocity(0);
                launcher2.setVelocity(0);
            }

            // ============ SERVO CONTROL ============
            if (gamepad2.left_bumper) {
                torqueArmServo.setPosition(SERVO_SHOOT_POS);

            } else if (gamepad2.right_bumper) {
                torqueArmServo.setPosition(SERVO_REST_POS);
                
            } else if (gamepad2.dpad_right) {
                torque2.setPosition(0.5);
                
            } else if (gamepad2.dpad_left) {
                torque2.setPosition(-0.5);
            }
            
            // ============ TELEMETRY ============
            // SPEED INFO (displayed first)
            telemetry.addLine("=== DRIVE SPEED ===");
            telemetry.addData("Speed", String.format("%.0f%% (%.2f)", speedMultiplier * 100, speedMultiplier));
            telemetry.addLine();
            
            // ODOMETRY INFO
            if (odometryInitialized && pos != null) {
                telemetry.addLine("=== ROBOT POSITION ===");
                telemetry.addData("X Position (inches)", "%.2f", pos.getX(DistanceUnit.INCH));
                telemetry.addData("Y Position (inches)", "%.2f", pos.getY(DistanceUnit.INCH));
                telemetry.addData("Heading (degrees)", "%.2f", pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Status", odometryComputer.getDeviceStatus());
                telemetry.addLine();
            } else {
                telemetry.addLine("=== ODOMETRY NOT AVAILABLE ===");
                telemetry.addLine();
            }
            
            // DRIVE INFO
            telemetry.addLine("Drive Velocity:");
            telemetry.addData("FL", "%.2f", frontLeft.getVelocity());
            telemetry.addData("FR", "%.2f", frontRight.getVelocity());
            telemetry.addData("BL", "%.2f", backLeft.getVelocity());
            telemetry.addData("BR", "%.2f", backRight.getVelocity());
            telemetry.addLine();
            
            // OTHER SUBSYSTEMS
            telemetry.addData("Intake Velocity", "%.2f", intakeMotor.getVelocity());
            telemetry.addData("Launcher1 Velocity", "%.2f", launcher1.getVelocity());
            telemetry.addData("Launcher2 Velocity", "%.2f", launcher2.getVelocity());
            telemetry.addData("Lift Velocity", "%.2f", liftMotor.getVelocity());
            telemetry.addData("Servo Pos", "%.2f", torqueArmServo.getPosition());
            telemetry.update();
        }

        // ====== STOP ALL MOTORS WHEN DONE ======
        stopAllMotors();
    }

    // ====== SPEED ADJUSTMENT HANDLER ======
    private void handleSpeedAdjustments() {
        // Circle (B): Toggle between 100% (turbo) and 50% (normal)
        if (gamepad1.circle && !lastCircle) {
            if (speedMultiplier >= 0.9) {
                speedMultiplier = 0.2; // Switch to normal speed
            } else {
                speedMultiplier = 1.0; // Switch to turbo
            }
        }
        lastCircle = gamepad1.circle;
    }

    // ====== HELPER METHOD ======
    private void stopAllMotors() {
        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);
        launcher1.setVelocity(0);
        launcher2.setVelocity(0);
        intakeMotor.setVelocity(0);
        liftMotor.setVelocity(0);
    }
    
    // ====== INITIALIZE PINPOINT ODOMETRY ======
    private void initializePinpoint() {
        try {
            telemetry.addData("Odometry", "Initializing goBUILDA Pinpoint...");
            telemetry.update();
            
            // Get the Pinpoint odometry computer from hardware map
            odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odometryComputer");
            
            telemetry.addData("Odometry", "Hardware found, configuring...");
            telemetry.update();
            
            // Set the odometry pod positions relative to the point that the odometry computer tracks around
            // MEASURE THESE VALUES ON YOUR ROBOT
            // X pod offset (forward/backward from tracking center)
            // Y pod offset (left/right from tracking center)
            odometryComputer.setOffsets(-84.0, -168.0, DistanceUnit.MM); // Using millimeters
            
            // Set the kind of pods used by your robot
            // goBUILDA odometry pods have a encoder resolution of 13.26291192 ticks per mm
            odometryComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            
            // Set the direction that each encoder counts
            // Test and reverse if needed
            odometryComputer.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            
            // Reset the Pinpoint
            odometryComputer.resetPosAndIMU();
            
            odometryInitialized = true;
            
            telemetry.addData("Odometry", "SUCCESS!");
            telemetry.addData("Device Status", odometryComputer.getDeviceStatus());
            telemetry.update();
            
        } catch (Exception e) {
            odometryInitialized = false;
            telemetry.addData("ERROR", "Failed to initialize Pinpoint!");
            telemetry.addData("Exception", e.getMessage());
            telemetry.addData("Check", "Is 'odometryComputer' configured in Robot Configuration?");
            telemetry.addData("Check", "Is it set as 'goBILDA Pinpoint' device type?");
            telemetry.update();
            sleep(5000); // Give time to read the error
        }
    }
}

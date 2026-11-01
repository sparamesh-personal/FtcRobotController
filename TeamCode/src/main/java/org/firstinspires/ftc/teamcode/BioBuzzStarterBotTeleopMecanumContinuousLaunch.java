

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MecanumTeleopContinuousLaunch", group = "StarterBot")
//@Disabled
public class BioBuzzStarterBotTeleopMecanumContinuousLaunch extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;


    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final double POLLEN_LAUNCHER_POWER = 0.40;
    public final double NECTAR_LAUNCHER_POWER = 0.58;
    private static final double SPIN_UP_TIME_SECONDS = 1.50;
    private static final double FEED_TIME_SECONDS = 1.00;
    private static final double FEED_POWER = 0.80;
    private static final double LAUNCHER_STICK_TRIM_RATE = 0.05;
    private static final double LAUNCHER_STICK_DEADBAND = 0.05;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        FEED
    }

    private LaunchState launchState = LaunchState.IDLE;
    private double launcherBasePower;
    private double launcherPower;
    private double launcherStickTrim;
    private boolean previousLeftBumper;
    private boolean previousRightBumper;
    private boolean previousBButton;
    private boolean continuousLaunchRunning;
    private String selectedArtifact = "None";
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private final ElapsedTime launcherTrimTimer = new ElapsedTime();


    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;
    private final ElapsedTime encoderSampleTimer = new ElapsedTime();
    private int previousLeftFrontPosition;
    private int previousRightFrontPosition;
    private int previousLeftBackPosition;
    private int previousRightBackPosition;
    private int previousIntakePosition;
    private double leftFrontTicksPerSecond;
    private double rightFrontTicksPerSecond;
    private double leftBackTicksPerSecond;
    private double rightBackTicksPerSecond;
    private double intakeTicksPerSecond;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         *
         * The launcher direction is REVERSED so it spins the opposite way. setDirection flips both
         * the motor output and the encoder reading, so getVelocity() stays positive.
         */
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPower(0);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        previousLeftFrontPosition = leftFrontDrive.getCurrentPosition();
        previousRightFrontPosition = rightFrontDrive.getCurrentPosition();
        previousLeftBackPosition = leftBackDrive.getCurrentPosition();
        previousRightBackPosition = rightBackDrive.getCurrentPosition();
        previousIntakePosition = intake.getCurrentPosition();
        encoderSampleTimer.reset();
        launcherTrimTimer.reset();
        launcherStickTrim = 0;
        previousLeftBumper = false;
        previousRightBumper = false;
        launchState = LaunchState.IDLE;
        selectedArtifact = "None";
        launcherBasePower = 0;
        launcherPower = 0;
        launcherStickTrim = 0;
        continuousLaunchRunning = false;
        previousBButton = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        updateEncoderRates();
        updateLauncherStickTrim();

        /*
         * Here we call a function called mecanumDrive. The mecanumDrive function takes the input from
         * the joysticks, and applies power to the drive motors to move the robot as requested
         * by the driver. Moving the left joystick forwards/back moves all motors forwards/back,
         * moving the right joystick left/right rotates the robot clockwise or counterclockwise,
         * and moving the left joystick left moves the motors in the right way to create a sideways
         * "strafe" movement. Combinations of these inputs can be used to create more complex maneuvers.
         * Note, moving the joystick forward on most gamepads results in a negative signal, so
         * we invert it before passing it to the function.
         */
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Set the intake power variable to equal the right trigger, minus the left trigger.
         * Each trigger outputs a signal from 0-1, with 0 as fully released, and 1 fully depressed.
         * This gives us proportional control of the intake speed. The speed increases as we pull
         * the right trigger further. It's occasionally helpful to be able to reverse the intake,
         * so we also factor in the left trigger. If the left trigger is fully depressed,
         * the intakePower variable will be -1. If the right trigger is fully depressed, the variable
         * will be 1. If the driver pulls both triggers, the intake will remain off.
         * We use this technique (creating a variable, and setting it to our control inputs) to
         * allow us to avoid setting the same motors/servos power more than once per loop. That can
         * create erratic behavior.
         */
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        launch();

        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("Drive LF", "p %.2f | %d ticks | %.0f ticks/s",
                leftFrontPower, leftFrontDrive.getCurrentPosition(), leftFrontTicksPerSecond);
        telemetry.addData("Drive RF", "p %.2f | %d ticks | %.0f ticks/s",
                rightFrontPower, rightFrontDrive.getCurrentPosition(), rightFrontTicksPerSecond);
        telemetry.addData("Drive LB", "p %.2f | %d ticks | %.0f ticks/s",
                leftBackPower, leftBackDrive.getCurrentPosition(), leftBackTicksPerSecond);
        telemetry.addData("Drive RB", "p %.2f | %d ticks | %.0f ticks/s",
                rightBackPower, rightBackDrive.getCurrentPosition(), rightBackTicksPerSecond);
        telemetry.addData("Drive Inputs", "forward %.2f | strafe %.2f | turn %.2f",
                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Intake Motor", "p %.2f | %d ticks | %.0f ticks/s",
                intake.getPower(), intake.getCurrentPosition(), intakeTicksPerSecond);
        telemetry.addData("Intake Servos", "left p %.2f | right p %.2f",
                leftIntakeServo.getPower(), rightIntakeServo.getPower());
        telemetry.addData("Windmill", "p %.2f", windmillServo.getPower());
        telemetry.addData("Launcher State", continuousLaunchRunning ? "CONTINUOUS" : "STOPPED");
        telemetry.addData("Selected Artifact", selectedArtifact);
        telemetry.addData("Launcher Power", "base %.2f | adjusted %.2f | motor %.2f",
                launcherBasePower, launcherPower, launcher.getPower());
        telemetry.addData("Continuous Launch", continuousLaunchRunning ? "ON (left=pollen, right=nectar, B=stop)" : "OFF");
        telemetry.addData("Launcher Stick Raw", "x %.2f | y %.2f",
                gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Launcher Stick Trim", "%.2f (hold up/down to tune)",
                launcherStickTrim);
        telemetry.addData("Launcher Motor", "mode %s | position %d | velocity %.0f ticks/s",
                launcher.getMode(), launcher.getCurrentPosition(), launcher.getVelocity());
        telemetry.addData("Spin-Up Progress", "%.2f of %.2f seconds",
                launchState == LaunchState.SPIN_UP ? spinUpTimer.seconds() : 0.0,
                SPIN_UP_TIME_SECONDS);
        telemetry.addData("Windmill", "power %.2f", windmillServo.getPower());
        telemetry.addData("Launch Timers", "spin-up %.2f s | feed %.2f s",
                launchState == LaunchState.SPIN_UP ? spinUpTimer.seconds() : 0.0,
                launchState == LaunchState.FEED ? feedTimer.seconds() : 0.0);
        telemetry.addData("Bumpers", "left %s | right %s",
                gamepad1.left_bumper ? "pressed" : "released",
                gamepad1.right_bumper ? "pressed" : "released");
        telemetry.addData("Intake Triggers", "in %.2f | reverse %.2f | final p %.2f",
                gamepad1.right_trigger, gamepad1.left_trigger, intake.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        launcher.setPower(0);
        windmillServo.setPower(0);
        launchState = LaunchState.IDLE;
        selectedArtifact = "None";
        launcherBasePower = 0;
        launcherPower = 0;
        launcherStickTrim = 0;
        continuousLaunchRunning = false;
        previousBButton = false;
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = -forward - strafe - rotate;
        rightFrontPower = -forward + strafe + rotate;
        leftBackPower = -forward + strafe - rotate;
        rightBackPower = -forward -  strafe + rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void updateLauncherStickTrim() {
        double elapsedSeconds = launcherTrimTimer.seconds();
        launcherTrimTimer.reset();
        if (elapsedSeconds <= 0) {
            return;
        }

        double stickY = gamepad1.right_stick_y;
        if (Math.abs(stickY) >= LAUNCHER_STICK_DEADBAND) {
            launcherStickTrim += -stickY * LAUNCHER_STICK_TRIM_RATE * elapsedSeconds;
            launcherStickTrim = Math.max(-0.50, Math.min(0.50, launcherStickTrim));
        }
    }

    private double getLauncherStickTrim() {
        return launcherStickTrim;
    }

    private double adjustedLauncherPower() {
        double adjustedPower = launcherBasePower + launcherStickTrim;
        return Math.max(0.0, Math.min(1.0, adjustedPower));
    }

    void launch() {
        boolean leftBumperPressed = gamepad1.left_bumper && !previousLeftBumper;
        boolean rightBumperPressed = gamepad1.right_bumper && !previousRightBumper;
        boolean bButtonPressed = gamepad1.b && !previousBButton;

        if (leftBumperPressed) {
            launcherBasePower = POLLEN_LAUNCHER_POWER;
            selectedArtifact = "Pollen";
            continuousLaunchRunning = true;
        } else if (rightBumperPressed) {
            launcherBasePower = NECTAR_LAUNCHER_POWER;
            selectedArtifact = "Nectar";
            continuousLaunchRunning = true;
        } else if (bButtonPressed) {
            continuousLaunchRunning = false;
            launcherBasePower = 0;
            launcherPower = 0;
            selectedArtifact = "None";
        }

        previousLeftBumper = gamepad1.left_bumper;
        previousRightBumper = gamepad1.right_bumper;
        previousBButton = gamepad1.b;

        if (continuousLaunchRunning) {
            launcherPower = adjustedLauncherPower();
            launcher.setPower(launcherPower);
            windmillServo.setPower(FEED_POWER);
            intakePower += 0.5;
        } else {
            launcher.setPower(0);
            windmillServo.setPower(gamepad1.a ? FEED_POWER : 0);
        }
    }

    private void updateEncoderRates() {
        double elapsedSeconds = encoderSampleTimer.seconds();
        if (elapsedSeconds <= 0) {
            return;
        }

        int leftFrontPosition = leftFrontDrive.getCurrentPosition();
        int rightFrontPosition = rightFrontDrive.getCurrentPosition();
        int leftBackPosition = leftBackDrive.getCurrentPosition();
        int rightBackPosition = rightBackDrive.getCurrentPosition();
        int intakePosition = intake.getCurrentPosition();

        leftFrontTicksPerSecond = (leftFrontPosition - previousLeftFrontPosition) / elapsedSeconds;
        rightFrontTicksPerSecond = (rightFrontPosition - previousRightFrontPosition) / elapsedSeconds;
        leftBackTicksPerSecond = (leftBackPosition - previousLeftBackPosition) / elapsedSeconds;
        rightBackTicksPerSecond = (rightBackPosition - previousRightBackPosition) / elapsedSeconds;
        intakeTicksPerSecond = (intakePosition - previousIntakePosition) / elapsedSeconds;

        previousLeftFrontPosition = leftFrontPosition;
        previousRightFrontPosition = rightFrontPosition;
        previousLeftBackPosition = leftBackPosition;
        previousRightBackPosition = rightBackPosition;
        previousIntakePosition = intakePosition;
        encoderSampleTimer.reset();
    }
}
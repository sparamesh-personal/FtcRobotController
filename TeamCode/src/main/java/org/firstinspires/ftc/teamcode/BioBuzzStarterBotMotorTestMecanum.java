/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumMotorTest", group = "StarterBot")
public class BioBuzzStarterBotMotorTestMecanum extends OpMode {

    private static final double MOTOR_POWER = 0.55;
    private static final double SERVO_POWER = 0.75;
    private static final double PHASE_SECONDS = 10.0;
    private static final double PAUSE_SECONDS = 1.0;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private CRServo windmillServo = null;

    private final ElapsedTime sequenceTimer = new ElapsedTime();
    private boolean sequenceStarted = false;
    private boolean sequenceFinished = false;
    private boolean xWasPressed = false;
    private boolean bWasPressed = false;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        stopAllHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Button", "Press X to start, B to stop/reset");
    }

    @Override
    public void start() {
        sequenceStarted = false;
        sequenceFinished = false;
        sequenceTimer.reset();
        xWasPressed = false;
        bWasPressed = false;
        stopAllHardware();
    }

    @Override
    public void loop() {
        boolean xPressed = gamepad1.x && !xWasPressed;
        boolean bPressed = gamepad1.b && !bWasPressed;

        xWasPressed = gamepad1.x;
        bWasPressed = gamepad1.b;

        if (bPressed) {
            sequenceStarted = false;
            sequenceFinished = false;
            sequenceTimer.reset();
            stopAllHardware();
            telemetry.addData("Sequence", "Stopped");
        }

        if (!sequenceStarted && !sequenceFinished && xPressed) {
            sequenceStarted = true;
            sequenceFinished = false;
            sequenceTimer.reset();
            telemetry.addData("Sequence", "Started");
        }

        if (!sequenceStarted) {
            stopAllHardware();
            telemetry.addData("Status", "Waiting for X button");
            telemetry.addData("Current", "Idle");
            telemetry.addData("Time", "0.00s");
            telemetry.addData("Button", "X = start, B = stop/reset");
            telemetry.update();
            return;
        }

        double elapsed = sequenceTimer.seconds();
        String currentDevice = "Idle";
        double totalCycleSeconds = (PHASE_SECONDS + PAUSE_SECONDS) * 6;
        double phasePosition = elapsed / totalCycleSeconds;

        if (elapsed < PHASE_SECONDS) {
            currentDevice = "Left Front Drive";
            setDrivePowers(MOTOR_POWER, 0.0, 0.0, 0.0);
            intake.setPower(0);
            windmillServo.setPower(0);
        } else if (elapsed < PHASE_SECONDS + PAUSE_SECONDS) {
            currentDevice = "Pause between tests";
            stopAllHardware();
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 2) {
            currentDevice = "Right Front Drive";
            setDrivePowers(0.0, MOTOR_POWER, 0.0, 0.0);
            intake.setPower(0);
            windmillServo.setPower(0);
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 2 + PAUSE_SECONDS) {
            currentDevice = "Pause between tests";
            stopAllHardware();
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 3) {
            currentDevice = "Left Back Drive";
            setDrivePowers(0.0, 0.0, MOTOR_POWER, 0.0);
            intake.setPower(0);
            windmillServo.setPower(0);
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 3 + PAUSE_SECONDS) {
            currentDevice = "Pause between tests";
            stopAllHardware();
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 4) {
            currentDevice = "Right Back Drive";
            setDrivePowers(0.0, 0.0, 0.0, MOTOR_POWER);
            intake.setPower(0);
            windmillServo.setPower(0);
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 4 + PAUSE_SECONDS) {
            currentDevice = "Pause between tests";
            stopAllHardware();
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 5) {
            currentDevice = "Intake";
            setDrivePowers(0.0, 0.0, 0.0, 0.0);
            intake.setPower(MOTOR_POWER);
            windmillServo.setPower(0);
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 5 + PAUSE_SECONDS) {
            currentDevice = "Pause between tests";
            stopAllHardware();
        } else if (elapsed < (PHASE_SECONDS + PAUSE_SECONDS) * 6) {
            currentDevice = "Windmill";
            setDrivePowers(0.0, 0.0, 0.0, 0.0);
            intake.setPower(0);
            windmillServo.setPower(SERVO_POWER);
        } else {
            currentDevice = "Complete";
            stopAllHardware();
            sequenceFinished = true;
            sequenceStarted = false;
        }

        telemetry.addData("Sequence", currentDevice);
        telemetry.addData("Elapsed Time", String.format("%.2f s", elapsed));
        telemetry.addData("Progress", String.format("%.0f%%", phasePosition * 100));
        telemetry.addData("Left Front", leftFrontDrive.getPower());
        telemetry.addData("Right Front", rightFrontDrive.getPower());
        telemetry.addData("Left Back", leftBackDrive.getPower());
        telemetry.addData("Right Back", rightBackDrive.getPower());
        telemetry.addData("Intake", intake.getPower());
        telemetry.addData("Windmill Servo", windmillServo.getPower());
        telemetry.addData("Button", "X = rerun, B = stop/reset");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAllHardware();
    }

    private void setDrivePowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }

    private void stopAllHardware() {
        setDrivePowers(0, 0, 0, 0);
        intake.setPower(0);
        windmillServo.setPower(0);
    }
}

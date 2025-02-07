
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaDriveToPoint;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import java.util.Locale;

@TeleOp(name = "PID TUNING", group = "Techtonics")
@Disabled
public class TT_PID_TUNING extends LinearOpMode {
    private TT_RobotHardware robot = new TT_RobotHardware(this);

    private double autonomousPower = .6;

    final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, calcXCoordinate(2000), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, calcXCoordinate(0), 0, AngleUnit.DEGREES, 0);

    final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, calcXCoordinate(0), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, calcXCoordinate(0), 0, AngleUnit.DEGREES, 180);

    private double calcXCoordinate(double xChange) {
        return xChange + 0;
    }

    boolean driveTuning = true;
    int xCoor = 0;
    int yCoor = 0;

    private static double xyTolerance = 20;
    private static double pGain = 0.0043;
    private static double dGain = 0.00001;
    private static double accel = 8.0;

    private static double GBxyTolerance = 20;
    private static double GBpGain = 0.008;
    private static double GBdGain = 0.00001;
    private static double GBaccel = 8.0;

    public double yawTolerance = 2; // 0.0349066;
    public double yawPGain = 5.0;
    public static double yawDGain = 0.0;
    public double yawAccel = 20.0;

    public double GByawTolerance = 2; //0.0349066;
    public double GByawPGain = 0.0;
    public static double GByawDGain = 0.0;
    public double GByawAccel = 20.0;

    enum StateMachine {
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4
    }

    @Override
    public void runOpMode() {
        robot.init();
        StateMachine stateMachine;
        stateMachine = StateMachine.DRIVE_TO_TARGET_1;
        robot.nav.setXYCoefficients(pGain, dGain, accel, MM, xyTolerance);
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double duration = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.odo.update();

            switch (stateMachine) {
                case DRIVE_TO_TARGET_1:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_1, autonomousPower, 3)) {
                        duration = timer.milliseconds();
                        timer.reset();
                        robot.nav.setXYCoefficients(pGain, dGain, accel, MM, xyTolerance);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_2, autonomousPower, 3)) {
                        duration = timer.milliseconds();
                        timer.reset();
                        robot.nav.setXYCoefficients(pGain, dGain, accel, MM, xyTolerance);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_3, autonomousPower, 3)) {
                        duration = timer.milliseconds();
                        timer.reset();
                        robot.nav.setYawCoefficients(yawPGain, yawPGain, yawPGain, AngleUnit.DEGREES, yawTolerance);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_4, autonomousPower, 3)) {
                        duration = timer.milliseconds();
                        timer.reset();
                        robot.nav.setYawCoefficients(yawPGain, yawPGain, yawPGain, AngleUnit.DEGREES, yawTolerance);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
            }

            if (gamepad1.dpad_left) {
                autonomousPower -= .01;
            } else if (gamepad1.dpad_right) {
                autonomousPower += .01;
            } else if (gamepad1.dpad_down) {
                if (driveTuning) {
                    xyTolerance -= .1;
                } else {
                    yawTolerance -= .1;
                }
            } else if (gamepad1.dpad_up) {
                if (driveTuning) {
                    xyTolerance += .1;
                } else {
                    yawTolerance += .1;
                }
            } else if (gamepad1.a) {
                if (driveTuning) {
                    dGain -= .000001;
                } else {
                    yawDGain -= .001;
                }
            } else if (gamepad1.b) {
                if (driveTuning) {
                    dGain += .000001;
                } else {
                    yawDGain += .001;
                }
            } else if (gamepad1.x) {
                if (driveTuning) {
                    pGain -= .0001;
                } else {
                    yawPGain -= .1;
                }
            } else if (gamepad1.y) {
                if (driveTuning) {
                    pGain += .0001;
                } else {
                    yawPGain += .1;
                }
            } else if (gamepad1.right_bumper) {
                driveTuning = false;
                stateMachine = StateMachine.DRIVE_TO_TARGET_3;
            } else if (gamepad1.left_bumper) {
                driveTuning = true;
                stateMachine = StateMachine.DRIVE_TO_TARGET_1;
            }

            robot.setDrivePower();
            telemetry.addLine("Mode: " + stateMachine.toString());
            telemetry.addLine("P: X/Y  D: A/B  T: DPU/DPD  Power: DPL/DPR   Mode: Bumpers");
            telemetry.addData("DRIVE: ", "P %1.1f  D %1.1f  Accel %1.1f  ", GBpGain * 1000, GBdGain * 100000, GBaccel, autonomousPower);
            telemetry.addData("DRIVE: ", "P %1.1f  D %1.1f  Accel %1.1f", pGain * 1000, dGain * 100000, accel, autonomousPower);
            telemetry.addData("DRIVE: ", "Tolerance  %1.1f  Original %1.1f  Power: %1.2f", xyTolerance, GBxyTolerance, autonomousPower);
            telemetry.addLine("");
            telemetry.addData("TURN: ", "P %1.3f  D %1.3f  Accel %1.1f", GByawPGain, GByawDGain, GByawAccel, autonomousPower);
            telemetry.addData("TURN: ", "P %1.3f  D %1.3f  Accel %1.1f", yawPGain, yawPGain, yawAccel, autonomousPower);
            telemetry.addData("TURN: ", "Tolerance  %1.1f  Original %1.1f  Power: %1.2f", yawTolerance, GByawTolerance, autonomousPower);
            telemetry.addData("Timing", "Duration %3.1f  Angle: %1.2f ", duration, robot.odo.getHeading() * (180/Math.PI));
            telemetry.update();
        }
    }
}


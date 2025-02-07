
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

@TeleOp(name = "PID TUNING ANGLES", group = "Techtonics")
@Disabled
public class TT_PID_TUNING_ANGLES extends LinearOpMode {
    private TT_RobotHardware robot = new TT_RobotHardware(this);

    private double autonomousPower = .6;

    final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 180);
    private Pose2D curTargetPose = TARGET_4;

    private double calcXCoordinate(double xChange) {
        return xChange + 0;
    }

    public double yawTolerance = 0.0349066;
    public double yawPGain = 5.0;
    public double yawDGain = 0.0;
    public double yawAccel = 20.0;

    public double GByawTolerance = 0.0349066;
    public double GByawPGain = 5.0;
    public double GByawDGain = 0.0;
    public double GByawAccel = 20.0;

    @Override
    public void runOpMode() {
        robot.init();
        boolean turnRight = true;
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double duration = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.odo.update();

            if (robot.nav.driveTo(robot.odo.getPosition(), curTargetPose, autonomousPower, 10)) {
                duration = timer.milliseconds();
                timer.reset();
                if (turnRight) {
                    curTargetPose = TARGET_3;
                } else {
                    curTargetPose = TARGET_4;
                }
                turnRight = !turnRight;
            }

            robot.nav.setYawCoefficients(yawPGain, yawPGain, yawAccel, AngleUnit.RADIANS, yawTolerance);
            if (gamepad1.dpad_left) {
                autonomousPower -= .01;
            } else if (gamepad1.dpad_right) {
                autonomousPower += .01;
            } else if (gamepad1.dpad_down) {
                yawTolerance -= .001;
            } else if (gamepad1.dpad_up) {
                yawTolerance += .001;
            } else if (gamepad1.a) {
                yawDGain -= .001;
            } else if (gamepad1.b) {
                yawDGain += .001;
            } else if (gamepad1.x) {
                yawPGain -= .1;
            } else if (gamepad1.y) {
                yawPGain += .1;
            } else if (gamepad1.left_bumper) {
                yawAccel -= .1;
            } else if (gamepad1.right_bumper) {
                yawAccel += .1;
            }

            robot.setDrivePower();
            telemetry.addLine("P: X/Y  D: A/B  T: DPAD-U/DPAD-D  Power: DPAD-L/DPAD-R   Mode: Bumpers");
            telemetry.addLine("");
            telemetry.addData("TURN: ", "P %1.3f  D %1.3f  Accel %1.1f", GByawPGain, GByawDGain, GByawAccel, autonomousPower);
            telemetry.addData("TURN: ", "P %1.3f  D %1.3f  Accel %1.1f", yawPGain, yawDGain, yawAccel, autonomousPower);
            telemetry.addData("TURN: ", "Tolerance  %1.5f  Original %1.5f  Power: %1.2f", yawTolerance, GByawTolerance, autonomousPower);
            telemetry.addData("Angle", "Target Angle: %1.2f  Angle: %1.2f Error: %1.3f", robot.nav.targetRadians, robot.nav.currentRadians, robot.nav.hError);
            telemetry.addData("Timing", "Duration %3.1f Inbounds: %s  Yaw Power: %1.3f", duration, robot.nav.inbounds, robot.nav.hOutput);
            telemetry.update();
        }
    }
}


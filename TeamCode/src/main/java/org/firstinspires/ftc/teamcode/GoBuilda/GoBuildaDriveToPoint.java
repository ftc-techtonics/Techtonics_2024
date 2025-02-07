package org.firstinspires.ftc.teamcode.GoBuilda;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class GoBuildaDriveToPoint {

    public enum DriveType {
        MECANUM
    }

    public enum DriveMotor {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    }

    private enum Direction {
        x,
        y,
        h
    }

    private enum InBounds {
        NOT_IN_BOUNDS,
        IN_X_Y,
        IN_HEADING,
        IN_BOUNDS
    }

    public InBounds inbounds = InBounds.NOT_IN_BOUNDS;
    public double hOutput = 0;
    public static double xyTolerance = 30; //20;
    public double yawTolerance = 0.08; //0.0349066;
    public double currentRadians = 0;
    public double targetRadians = 0;

    private static double pGain = 0.0043;
    private static double dGain = 0.00001;
    private static double accel = 8.0;

    public double yawPGain = 5.0;
    public static double yawDGain = 0.0;
    public double yawAccel = 3;//20.0;
    public double hError = 0;

    private double leftFrontMotorOutput = 0;
    private double rightFrontMotorOutput = 0;
    ;
    private double leftBackMotorOutput = 0;
    private double rightBackMotorOutput = 0;

    private final ElapsedTime holdTimer = new ElapsedTime();
    private final ElapsedTime PIDTimer = new ElapsedTime();

    private final PIDLoop xPID = new PIDLoop();
    private final PIDLoop yPID = new PIDLoop();
    private final PIDLoop hPID = new PIDLoop();
    private LinearOpMode myOpMode;

    public GoBuildaDriveToPoint(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void setXYCoefficients(double p, double d, double acceleration, DistanceUnit unit, double tolerance) {
        pGain = p;
        dGain = d;
        accel = acceleration;
        xyTolerance = unit.toMm(tolerance);
    }

    public void setYawCoefficients(double p, double d, double acceleration, AngleUnit unit, double tolerance) {
        yawPGain = p;
        yawDGain = d;
        yawAccel = acceleration;
        yawTolerance = unit.toRadians(tolerance);
    }

    public double getMotorPower(DriveMotor driveMotor) {
        if (driveMotor == DriveMotor.LEFT_FRONT) {
            return leftFrontMotorOutput;
        } else if (driveMotor == DriveMotor.RIGHT_FRONT) {
            return rightFrontMotorOutput;
        } else if (driveMotor == DriveMotor.LEFT_BACK) {
            return leftBackMotorOutput;
        } else {
            return rightBackMotorOutput;
        }
    }

    public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
        boolean atTarget;

        double xPWR = calculatePID(currentPosition, targetPosition, Direction.x);
        double yPWR = calculatePID(currentPosition, targetPosition, Direction.y);
        hOutput = calculatePID(currentPosition, targetPosition, Direction.h);

        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);

        double xOutput = (xPWR * cosine) + (yPWR * sine);
        double yOutput = (xPWR * sine) - (yPWR * cosine);

        inbounds = inBounds(currentPosition, targetPosition);
        //if (inbounds == InBounds.IN_BOUNDS) {
        //    hOutput = 0;
        //}


        calculateMecanumOutput(xOutput * power, yOutput * power, hOutput * power);

        if (inbounds == InBounds.IN_BOUNDS) {
            atTarget = true;
        } else {
            holdTimer.reset();
            atTarget = false;
        }
        if (atTarget && holdTimer.time() > holdTime) {
            return true;
        }
        return false;
    }

    private void calculateMecanumOutput(double forward, double strafe, double yaw) {
        double leftFront = forward - -strafe - yaw;
        double rightFront = forward + -strafe + yaw;
        double leftBack = forward + -strafe - yaw;
        double rightBack = forward - -strafe + yaw;

        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));

        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }

        leftFrontMotorOutput = leftFront;
        rightFrontMotorOutput = rightFront;
        leftBackMotorOutput = leftBack;
        rightBackMotorOutput = rightBack;
    }

    private double calculatePID(Pose2D currentPosition, Pose2D targetPosition, Direction direction) {
        if (direction == Direction.x) {
            double xError = targetPosition.getX(MM) - currentPosition.getX(MM);
            return xPID.calculateAxisPID(xError, pGain, dGain, accel, PIDTimer.seconds());
        }
        if (direction == Direction.y) {
            double yError = targetPosition.getY(MM) - currentPosition.getY(MM);
            return yPID.calculateAxisPID(yError, pGain, dGain, accel, PIDTimer.seconds());
        }
        if (direction == Direction.h) {
            hError = calculateHError(targetPosition.getHeading(AngleUnit.RADIANS), currentPosition.getHeading(AngleUnit.RADIANS));
            return hPID.calculateTurnPID(hError, yawPGain, yawDGain, yawAccel, PIDTimer.seconds());
        }
        return 0;
    }

    private InBounds inBounds(Pose2D currPose, Pose2D trgtPose) {
        boolean xInBounds = currPose.getX(MM) > (trgtPose.getX(MM) - xyTolerance) && currPose.getX(MM) < (trgtPose.getX(MM) + xyTolerance);
        boolean yInBounds = currPose.getY(MM) > (trgtPose.getY(MM) - xyTolerance) && currPose.getY(MM) < (trgtPose.getY(MM) + xyTolerance);
        boolean hInBounds = (Math.abs(hError) < yawTolerance);

        if (xInBounds && yInBounds && hInBounds) {
            return InBounds.IN_BOUNDS;
        } else if (xInBounds && yInBounds) {
            return InBounds.IN_X_Y;
        } else if (hInBounds) {
            return InBounds.IN_HEADING;
        } else {
            return InBounds.NOT_IN_BOUNDS;
        }
    }

    private double calculateHError(double targetRadiansPARM, double currentRadiansPARM) {
        currentRadians = currentRadiansPARM;
        targetRadians = targetRadiansPARM;
        double error = targetRadians - currentRadians;
        //=IF((C29-D29)>180,-(360-(C29-D29)),(C29-D29))
        //=IF((C29-D29)<-180,(360+(C29-D29)),(C29-D29))
        if (error < -Math.PI) {
            error = ((2 * Math.PI) + error);
        } else if (error > Math.PI) {
            error = -((2 * Math.PI) - error);
        }
        return error;
    }

    public double calculateTargetHeading(Pose2D currPose, Pose2D trgtPose) {
        double xDelta = trgtPose.getX(MM) - currPose.getX(MM);
        double yDelta = trgtPose.getY(MM) - currPose.getY(MM);

        if (Math.abs(xDelta) > xyTolerance || Math.abs(yDelta) > xyTolerance) {
            return Math.atan2(yDelta, xDelta);
        } else {
            return currPose.getHeading(RADIANS);
        }

    }
}

class PIDLoop {
    private double previousError;
    private double previousTime;
    private double previousOutput;

    private double errorR;

    public double calculateTurnPID(double error, double pGain, double dGain, double accel, double currentTime) {
        previousError = Math.abs(error);
        if (Math.abs(error) < (Math.PI / 90)) {
            return 0;
        } else if (Math.abs(error) < (Math.PI / 90) * 20) {
            return 0.2 * (previousError/error) ;
        } else {
            return calculateAxisPID(error,pGain,dGain,accel,currentTime);
            //return error * accel;
        }
    }

    public double calculateAxisPID(double error, double pGain, double dGain, double accel, double currentTime) {
        double p = error * pGain;
        double cycleTime = currentTime - previousTime;
        double errorDelta = (previousError - error);

        errorDelta = Math.abs(previousError) - Math.abs(error);
        if (error < 1) {
            errorDelta *= -1;
        }
                /*
        if (previousError > 0 && error < 0) {
            errorDelta = previousError + error;
        } else if (previousError < 0 && error > 0) {
            errorDelta = previousError + error;
        }
                 */

        double d = dGain * errorDelta / (cycleTime);
        double output = p + d;
        double dV = cycleTime * accel;

        double max = Math.abs(output);
        if (max > 1.0) {
            output /= max;
        }

        if ((output - previousOutput) > dV) {
            output = previousOutput + dV;
        } else if ((output - previousOutput) < -dV) {
            output = previousOutput - dV;
        }

        previousOutput = output;
        previousError = error;
        previousTime = currentTime;

        errorR = error;

        return output;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

@Autonomous(name = "Auto: Right Side", group = "Techtonics")
//@Disabled

public class TT_AutonomousRightSide_Verson2 extends LinearOpMode {
    TT_RobotHardware robot = new TT_RobotHardware(this);
    double autonomousFullPower = 1;
    double autonomousLowPower = 0.6;
    double startingPoint = 0;   // just right of Center
    int dropCount = 0;
    boolean Four_Specimans = true;
    boolean dropComplete = false;
    boolean firstTime = true;

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_SPECIMEN_DROP_PREP,
        DRIVE_TO_SPECIMEN_DROP,
        DRIVE_TO_BLOCK_1_PREP,
        DRIVE_TO_BLOCK_1_A,
        DRIVE_TO_BLOCK_1_B,
        DRIVE_TO_BLOCK_1_C,
        DRIVE_TO_BLOCK_2_A,
        DRIVE_TO_BLOCK_2_B,
        DRIVE_TO_BLOCK_2_C,
        DRIVE_TO_BLOCK_3_A,
        DRIVE_TO_BLOCK_3_B,
        DRIVE_TO_BLOCK_3_C,
        DRIVE_TO_SPECIMEN_PICKUP_PREP,
        DRIVE_TO_SPECIMEN_PICKUP,
        DRIVE_TO_PARKING,
    }

    // REMEMBER TO ADD 50 DISTANCE TO Y AXIS MOVING RIGHT
    int MAX_FORWARD_POSITION = -1100;

    final static double specimanYCoordinate = 0;
    final static double specimanXCoordinate = -740;

    final Pose2D TARGET_BLOCK_1_A_Prep = new Pose2D(DistanceUnit.MM, calcXCoordinate(specimanXCoordinate + 200), 375, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_1_A = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 690, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_1_B = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION - 100), 890, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_1_C = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 890, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_2_A = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 890, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_2_B = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION - 100), 1190, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_2_C = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 1190, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_3_A = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 1290, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_3_B = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 1390, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_3_C = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 1390, AngleUnit.DEGREES, 0);

    final Pose2D TARGET_SPECIMEN_PREPARE = new Pose2D(DistanceUnit.MM, calcXCoordinate(-200), 690, AngleUnit.DEGREES, 180);
    final Pose2D TARGET_SPECIMEN_PICKUP = new Pose2D(DistanceUnit.MM, calcXCoordinate(0), 690, AngleUnit.DEGREES, 180);

    final Pose2D TARGET_SPECIMEN_DROP_PREPARE_1 = new Pose2D(DistanceUnit.MM, specimanXCoordinate + 150, specimanYCoordinate, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_PREPARE_2 = new Pose2D(DistanceUnit.MM, specimanXCoordinate + 150, specimanYCoordinate - 50, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_PREPARE_3 = new Pose2D(DistanceUnit.MM, specimanXCoordinate + 150, specimanYCoordinate - 100, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_PREPARE_4 = new Pose2D(DistanceUnit.MM, specimanXCoordinate + 150, specimanYCoordinate - 150, AngleUnit.DEGREES, 0);

    final Pose2D TARGET_SPECIMEN_DROP_1 = new Pose2D(DistanceUnit.MM, specimanXCoordinate, specimanYCoordinate, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_2 = new Pose2D(DistanceUnit.MM, specimanXCoordinate, specimanYCoordinate - 50, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_3 = new Pose2D(DistanceUnit.MM, specimanXCoordinate, specimanYCoordinate - 100, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_4 = new Pose2D(DistanceUnit.MM, specimanXCoordinate, specimanYCoordinate - 150, AngleUnit.DEGREES, 0);

    final Pose2D PARKING = new Pose2D(DistanceUnit.MM, -150, 800, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {
        runGeneralOpMode();
    }

    public void runGeneralOpMode() {
        //telemetry.addData("auto:", "4 specimens %s", Four_Specimans);
        robot.init();
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;
        // Wait for the game to start (driver presses START)

        //public double yawTolerance = 0.04; //0.0349066;
        robot.nav.yawTolerance = 0.2; // Normally 0.0349066;
        robot.nav.yawAccel = 10;      // Normally 20;
        robot.nav.yawPGain = 3;       // Normally 5;

        waitForStart();
        resetRuntime();
        robot.extensionArm.setTargetPosition(robot.EXT_ArmMidHeight);

        while (opModeIsActive()) {
            robot.odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP_PREP;
                    robot.setLiftPosition(robot.liftHeightSpecimenDrop);
                    break;
                case DRIVE_TO_BLOCK_1_PREP:
                    if (driveToTarget(TARGET_BLOCK_1_A_Prep, autonomousFullPower, 0, "back right to position for first block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_1_A;
                        robot.setLiftPosition(robot.liftHeightMin);
                    }
                    break;
                case DRIVE_TO_BLOCK_1_A:
                    if (driveToTarget(TARGET_BLOCK_1_A, autonomousFullPower, 0, "Forward to first block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_1_B;
                    }
                    break;
                case DRIVE_TO_BLOCK_1_B:
                    if (driveToTarget(TARGET_BLOCK_1_B, autonomousFullPower, 0, "Move Right to first block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_1_C;
                    }
                    break;
                case DRIVE_TO_BLOCK_1_C:
                    if (driveToTarget(TARGET_BLOCK_1_C, autonomousFullPower, 0, "Move back with first block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_2_A;
                    }
                    break;
                case DRIVE_TO_BLOCK_2_A:
                    if (driveToTarget(TARGET_BLOCK_2_A, autonomousFullPower, 0, "Forward to second block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_2_B;
                    }
                    break;

                case DRIVE_TO_BLOCK_2_B:
                    if (driveToTarget(TARGET_BLOCK_2_B, autonomousFullPower, 0, "Move Right to second block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_2_C;
                    }
                    break;
                case DRIVE_TO_BLOCK_2_C:
                    if (driveToTarget(TARGET_BLOCK_2_C, autonomousFullPower, 0, "Move back with second block")) {
                        if (Four_Specimans) {
                            // Skip last block and instead go for putting on 3 more specimens.
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                        } else {
                            stateMachine = StateMachine.DRIVE_TO_BLOCK_3_A;
                        }
                    }
                    break;
                case DRIVE_TO_BLOCK_3_A:
                    if (driveToTarget(TARGET_BLOCK_3_A, autonomousFullPower, 0, "Forward to third block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_3_B;
                    }
                    break;

                case DRIVE_TO_BLOCK_3_B:
                    if (driveToTarget(TARGET_BLOCK_3_B, autonomousFullPower, 0, "Move Right to third block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_3_C;
                    }
                    break;

                case DRIVE_TO_BLOCK_3_C:
                    if (driveToTarget(TARGET_BLOCK_3_C, autonomousFullPower, 0, "Move back with third block")) {
                        stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                    }
                    break;
                case DRIVE_TO_SPECIMEN_PICKUP_PREP:
                    robot.setLiftPosition(0);
                    dropComplete = false;
                    if (driveToTarget(TARGET_SPECIMEN_PREPARE, autonomousFullPower, 0.5, "Prepare for Specimen Pickup")) {
                        stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP;
                    }
                    break;
                case DRIVE_TO_SPECIMEN_PICKUP:
                    if (driveToTarget(TARGET_SPECIMEN_PICKUP, 0.6, 0.1, "Pickup Specimen from wall")) {
                        if ((Four_Specimans && dropCount < 4) || (dropCount < 3)) {
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP_PREP;
                        }
                        // ELSE this is the end of the program.   Don't drop any more and just park here.
                    }
                    break;

                case DRIVE_TO_SPECIMEN_DROP_PREP:
                    if (!dropComplete) {
                        robot.setLiftPosition(robot.liftHeightSpecimenDrop);
                    }

                    if (dropCount == 0 || firstTime) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP_PREPARE_1, autonomousLowPower, 0, "Prepare for Drop of Specimen")) {
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP;
                            if (dropCount == 1) {
                                firstTime = false;
                                stateMachine = StateMachine.DRIVE_TO_BLOCK_1_PREP;
                            }
                        }
                    } else if (dropCount == 1) {
                        if ((driveToTarget(TARGET_SPECIMEN_DROP_PREPARE_2, autonomousFullPower, 0, "Prepare for Drop of Specimen"))) {
                            if (dropComplete) {
                                stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                            } else {
                                stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP;
                            }
                        }
                    } else if (dropCount == 2) {
                        if ((driveToTarget(TARGET_SPECIMEN_DROP_PREPARE_3, autonomousFullPower, 0, "Prepare for Drop of Specimen"))) {
                            if (dropComplete) {
                                stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                            } else {
                                stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP;
                            }
                        }
                    } else if (dropCount > 2) {
                        if ((driveToTarget(TARGET_SPECIMEN_DROP_PREPARE_4, autonomousFullPower, 0, "Prepare for Drop of Specimen"))) {
                            if (dropComplete) {
                                stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                            } else {
                                stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP;
                            }
                        }
                    }
                    break;
                case DRIVE_TO_SPECIMEN_DROP:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */

                    if (dropCount == 0) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP_1, autonomousLowPower, 0.3, "Driving to Drop Position 1")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP_PREP;
                        }
                    } else if (dropCount == 1) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP_2, autonomousLowPower, 0.1, "Driving to Drop Position 2")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP_PREP;
                        }
                    } else if (dropCount == 2) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP_3, autonomousLowPower, 0.1, "Driving to Drop Position 3")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP_PREP;
                        }
                    } else if (dropCount == 3) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP_4, autonomousLowPower, 0.1, "Driving to Drop Position 4")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_PARKING;
                        }
                    }
                    break;
                case DRIVE_TO_PARKING:
                    robot.setLiftPosition(0);
                    driveToTarget(PARKING, autonomousFullPower, 5, "Driving to Parking Position");
                    break;
            }

            robot.setDrivePower();
            robot.displayTelemetry();
        }
    }

    private void dropSpecimen() {
        dropComplete = true;
        dropCount = dropCount + 1;
        robot.setLiftPosition(robot.liftHeightSpecimenDrop - 330);
        sleep(200);
    }

    private boolean driveToTarget(Pose2D target, double drivePower, double holdTime, String message) {
        telemetry.addLine(message);
        robot.targetPosition = target;
        return (robot.nav.driveTo(robot.odo.getPosition(), target, drivePower, holdTime));
    }

    private double calcXCoordinate(double xChange) {
        return xChange + startingPoint;
    }
}

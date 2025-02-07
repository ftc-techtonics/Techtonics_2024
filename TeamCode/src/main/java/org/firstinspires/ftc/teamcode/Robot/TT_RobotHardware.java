/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaDriveToPoint;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaPinpointDriver;

import java.util.Locale;

public class TT_RobotHardware {

    public LinearOpMode myOpMode = null;

    //                                                  DRIVE TRAIN
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public double MaxPowerAdjustment = 1;   // 1 is full power.  going up reduces power

    //                                          ODOMETRY & AUTO NAVIGATION
    public GoBuildaPinpointDriver odo;
    public GoBuildaDriveToPoint nav; //OpMode member for the point-to-point navigation class
    public Pose2D targetPosition;
    public Pose2D targetPositionPrep;
    public Pose2D targetPositionPrepRight;
    public Pose2D startPosition;
    public Pose2D startPositionPrep;

    final Pose2D targetPositionRight = new Pose2D(DistanceUnit.MM, -150, 400, AngleUnit.DEGREES, 180);

    public boolean startPrepRequired = false;
    public boolean fullAutonomous = false;

    //                                                  AI CAMERA
    private Limelight3A camera;
    private double target_X_Coor = 0;
    private double target_Y_Coor = 0;
    private Pose3D botPose = null;

    //                                                  LIFT SYSTEM
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;

    public int liftHeight = 0;
    public int liftHeightMin = 0;
    public int liftHeightSpecimenDrop = 1265;
    public int liftHeightMax = 1465;

    private boolean liftArmUp = true;
    private boolean liftArmButtonPress = false;

    public DigitalChannel liftSafetyButton = null;
    private boolean liftEnabled = false;

    public int liftSafetyThreshold = 50;
    public int liftOffset = 0;
    public double effectivePower = 0;
    public double liftPowerMax = 1;

    //                                                  LIFT ARM
    public ScaledServo liftArm = null;
    public final double liftArmOutOfTheWay = .6;
    public final double LIFT_ARM_UP = 1;
    public final double LIFT_ARM_READY_TO_DROP = .6;
    public final double LIFT_ARM_DOWN = .1;

    //                                              EXTENSION MECHANSIMS
    public ScaledServo extension = null;
    public ScaledServo extensionArm = null;
    public ScaledServo extensionSpin = null;
    public ScaledServo extensionGripper = null;

    public double EXT_ArmDropHeight = 0.1;
    public double EXT_ArmDropHeightSpecimen = 0;
    public double EXT_ArmInitPosition = 0.20;
    public double EXT_ArmMidHeight = 0.30;
    public double EXT_ArmPickupReadyHeight = 0.87;
    public double EXT_ArmPickupHeight = 1;

    enum ExtensionArmState {
        PICKUP_POSITION,
        DROP_POSITION,
        DROP_POSITION_SPECIMEN,
        READY_TO_PICKUP,
        INITIALIZE_POSITION,
        MID_HEIGHT
    }

    public double EXT_GripperOpen = 1;
    public double EXT_GripperClose = 0;
    public double EXT_GripperReadyToClose = .7;

    public double EXT_SpinCenter = .5;

    //                                                  LED LIGHT
    public ScaledServo light = null;
    public double LIGHT_RED = 0.279;
    public double LIGHT_GREEN = 0.5;

    //                                                  GESTURE CONTROLS
    boolean gesturesSide = false;
    boolean gesturesChange = false;


    enum DropBlueRedGestureState {
        DRIVE_TO_DROP_POSITION_START,
        DRIVE_TO_DROP_POSITION,
        DROP_SAMPLE_IN_OBSERVATION_ZONE,
        RETURN_TO_START_PREP,
        RETURN_TO_START
    }

    enum DropYellowGestureState {
        DRIVE_TO_DROP_POSITION_START,
        DRIVE_TO_DROP_POSITION,
        TRANSFER_TO_LIFT_BOX,
        RAISE_LIFT,
        DROP_SAMPLE_IN_SCORING_BOX,
        RETRACT_ARM,
        DROP_LIFT,
        RETURN_TO_START_PREP,
        RETURN_TO_START
    }

    private DropYellowGestureState dropGestureState;
    private DropBlueRedGestureState dropBlueRedState;
    public boolean pickupGestureInitiated = false;
    private ElapsedTime pickupGestureTimer = new ElapsedTime();
    public boolean dropSampleGestureInitatied = false;
    public boolean dropSpecimanSampleGestureInitatied = false;
    public int gestureStep = 0;
    private boolean gestureAtPosition = false;
    private ElapsedTime dropSampleGestureTimer = new ElapsedTime();
    private double lastGestureTime;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public TT_RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     */
    public void init() {
        //                                              INITIALIZE DRIVE TRAIN
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "Left Front");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "Left Back");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "Right Front");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "Right Back");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //                                              INITIALIZE ODOMETRY & NAVIGATION
        odo = myOpMode.hardwareMap.get(GoBuildaPinpointDriver.class, "odo");
        odo.setOffsets(-145.0, -200.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBuildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBuildaPinpointDriver.EncoderDirection.FORWARD, GoBuildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        nav = new GoBuildaDriveToPoint(myOpMode);

        //                                              INITIALIZE AI CAMERA
        camera = myOpMode.hardwareMap.get(Limelight3A.class, "Camera");
        camera.pipelineSwitch(0);                 // Reflective color pipeline
        myOpMode.telemetry.setMsTransmissionInterval(11);
        camera.start();                                 // Starts polling for data.

        //                                              INITIALIZE LIFT SYSTEM
        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "left lift");
        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "right lift");
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftHeight = leftLift.getCurrentPosition();
        rightLift.setTargetPosition(liftHeight);
        leftLift.setTargetPosition(liftHeight);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftSafetyButton = myOpMode.hardwareMap.get(DigitalChannel.class, "Lift Safety Button");
        liftSafetyButton.setMode(DigitalChannel.Mode.INPUT);
        if (!liftSafetyButton.getState()) {
            liftEnabled = true;
        }

        liftArm = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Lift Arm"), "Lift Arm", .4, 1);
        liftArm.setTargetPosition(LIFT_ARM_DOWN);

        //                                              INITIALIZE EXTENSION SYSTEM
        extension = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension"), "Extension", 0.0, 0.6);
        extensionArm = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension Arm"), "Extension Arm", 0.75, .95);
        extensionSpin = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension Spin"), "Extension Spin", .80, 1);
        extensionGripper = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension Gripper"), "Extension Gripper", 0.5, 1);

        extension.setTargetPosition(1.0);
        extensionArm.setTargetPosition(EXT_ArmInitPosition);
        extensionSpin.setTargetPosition(EXT_SpinCenter);
        extensionGripper.setTargetPosition(EXT_GripperClose);

        light = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Light"), "Light", 0.0, 1.0);

        displayTelemetry();
    }

    public void checkLiftPosition() {
        if (liftSafetyButton.getState()) {
            light.setTargetPosition(LIGHT_RED);
        } else {
            light.setTargetPosition(LIGHT_GREEN);
        }
    }

    public void savePositionAndSide() {
        if (myOpMode.gamepad1.left_stick_button) {
            targetPosition = odo.getPosition();
            targetPositionPrep = new Pose2D(DistanceUnit.MM, targetPosition.getX(DistanceUnit.MM) - 100, targetPosition.getY(DistanceUnit.MM) + 100, AngleUnit.DEGREES, targetPosition.getHeading(AngleUnit.DEGREES));
        } else if (myOpMode.gamepad1.dpad_left) {
            if (gesturesChange == false) {
                gesturesSide = !gesturesSide;
                gesturesChange = true;
            }
        } else {
            gesturesChange = false;
        }
    }

    private void setExtensionArmPosition(ExtensionArmState armState) {
        switch (armState) {
            case INITIALIZE_POSITION:
                extensionArm.setTargetPosition(EXT_ArmInitPosition);
                break;
            case DROP_POSITION:
                extensionArm.setTargetPosition(EXT_ArmDropHeight);
                break;
            case MID_HEIGHT:
                extensionArm.setTargetPosition(EXT_ArmMidHeight);
                break;
            case PICKUP_POSITION:
                extensionArm.setTargetPosition(EXT_ArmPickupHeight);
                break;
            case READY_TO_PICKUP:
                extensionArm.setTargetPosition(EXT_ArmPickupReadyHeight);
                break;
            case DROP_POSITION_SPECIMEN:
                extensionArm.setTargetPosition(EXT_ArmDropHeightSpecimen);
                break;

        }
    }

    public void moveExtensionArm() {
        if (myOpMode.gamepad1.b) {
            // positioned over blocks for easy viewing
            setExtensionArmPosition(ExtensionArmState.READY_TO_PICKUP);
            extensionGripper.setTargetPosition(EXT_GripperReadyToClose);
        } else if (myOpMode.gamepad1.a) {
            // pickup the block
            if (pickupGestureInitiated == false) {
                pickupGestureInitiated = true;
                pickupGestureTimer.reset();
                setExtensionArmPosition(ExtensionArmState.PICKUP_POSITION);
                extensionGripper.setTargetPosition(EXT_GripperReadyToClose);
            }
        } else if (myOpMode.gamepad1.x) {
            // bring arm back for returning block.
            //transferGestureInitiated = true;
            //transferGestureTimer.reset();
            extensionGripper.setTargetPosition(EXT_GripperClose);
            if (leftLift.getTargetPosition() != liftHeightMin) {
                setExtensionArmPosition(ExtensionArmState.DROP_POSITION_SPECIMEN);
            } else {
                setExtensionArmPosition(ExtensionArmState.DROP_POSITION);
            }
            extensionSpin.setTargetPosition(EXT_SpinCenter);
            extension.setTargetPosition(1);
        } else if (myOpMode.gamepad1.y) {
            if (!dropSampleGestureInitatied) {
                dropGestureState = DropYellowGestureState.TRANSFER_TO_LIFT_BOX;
                dropSampleGestureInitatied = true;
                dropSampleGestureTimer.reset();
            }
        } else if (myOpMode.gamepad1.right_stick_button) {
            if (gesturesSide) {
                if (!dropSpecimanSampleGestureInitatied) {
                    dropBlueRedState = dropBlueRedState.DRIVE_TO_DROP_POSITION_START;
                    startPosition = odo.getPosition();
                    dropSampleGestureTimer.reset();
                    dropSpecimanSampleGestureInitatied = true;
                    lastGestureTime = 0;
                }
            } else if (!dropSampleGestureInitatied && targetPosition != null) {
                dropGestureState = DropYellowGestureState.DRIVE_TO_DROP_POSITION_START;
                startPosition = odo.getPosition();
                dropSampleGestureTimer.reset();
                dropSampleGestureInitatied = true;
                lastGestureTime = 0;
            }
        }
        if (pickupGestureInitiated) {
            if (pickupGestureTimer.seconds() > .2) {
                extensionGripper.setTargetPosition(EXT_GripperClose);
            }
            if (pickupGestureTimer.seconds() > .3) {
                setExtensionArmPosition(ExtensionArmState.READY_TO_PICKUP);
                pickupGestureInitiated = false;
            }
        }
    }

    private boolean checkForGestureCancellation() {

        boolean cancel = false;
        if (fullAutonomous && ((Math.abs(myOpMode.gamepad1.left_stick_y) > .2)
                || (Math.abs(myOpMode.gamepad1.left_stick_x) > .2)
                || (Math.abs(myOpMode.gamepad1.right_stick_x) > .2))) {
            cancel = true;
        }

        if (cancel) {
            dropSampleGestureInitatied = false;
            dropSpecimanSampleGestureInitatied = false;
            fullAutonomous = false;
            gestureStep = 0;
            lastGestureTime = 0;
            gestureAtPosition = false;
            setLiftPosition(liftHeightMin);
            liftArm.setTargetPosition(LIFT_ARM_DOWN);
        }
        return cancel;
    }

    public void processDropSampleGesture() {
        if (checkForGestureCancellation()) {
            return;
        }

        odo.update();
        double curTime = dropSampleGestureTimer.milliseconds();
        // Open Gripper - Drop block into Transfer box
        switch (dropGestureState) {
            case DRIVE_TO_DROP_POSITION_START:
                fullAutonomous = true;
                if (nav.driveTo(odo.getPosition(), targetPositionPrep, 1, 0)) {
                    //lastGestureTime = curTime;
                    gestureAtPosition = true;
                }
                setDrivePower();

                extension.setTargetPosition(1);
                extensionSpin.setTargetPosition((EXT_SpinCenter));

                if (gestureStep == 0) { // && extensionGripper.getTargetPosition() == EXT_GripperClose && extensionArm.getTargetPosition() != EXT_ArmDropHeight) {
                    setExtensionArmPosition(ExtensionArmState.DROP_POSITION);
                    lastGestureTime = curTime;
                    gestureStep += 1;
                }

                if ((gestureStep == 1) && (curTime - lastGestureTime) > 1000) {
                    extensionGripper.setTargetPosition(EXT_GripperReadyToClose);
                    lastGestureTime = curTime;
                    gestureStep += 1;
                }

                if ((gestureStep == 2) && (curTime - lastGestureTime) > 50) {
                    setExtensionArmPosition(ExtensionArmState.MID_HEIGHT);
                    lastGestureTime = curTime;
                    if (gestureAtPosition) {
                        dropGestureState = DropYellowGestureState.DRIVE_TO_DROP_POSITION;
                    }
                }

                break;
            case DRIVE_TO_DROP_POSITION:
                if (targetPosition != null && nav.driveTo(odo.getPosition(), targetPosition, .7, .2)) {
                    lastGestureTime = curTime;
                    dropGestureState = DropYellowGestureState.RAISE_LIFT;
                }
                setDrivePower();
                break;
            case TRANSFER_TO_LIFT_BOX:
                fullAutonomous = false;
                extensionGripper.setTargetPosition(EXT_GripperReadyToClose);  // Open Gripper to drop sample
                lastGestureTime = curTime;
                dropGestureState = DropYellowGestureState.RAISE_LIFT;
                break;
            case RAISE_LIFT:
                if ((curTime - lastGestureTime) > 100) {
                    // Lower arm to move it out of the way of lift and bring extension in
                    setExtensionArmPosition(ExtensionArmState.READY_TO_PICKUP);
                    extensionGripper.setTargetPosition(EXT_GripperClose);
                    extension.setTargetPosition(1);
                    // raise lift in preparation for dropping into scoring box
                    liftPowerMax = 1;
                    setLiftPosition(liftHeightMax);
                    liftArm.setTargetPosition(LIFT_ARM_READY_TO_DROP);

                    lastGestureTime = curTime;
                    dropGestureState = DropYellowGestureState.DROP_SAMPLE_IN_SCORING_BOX;
                }
                break;
            case DROP_SAMPLE_IN_SCORING_BOX:
                if ((curTime - lastGestureTime) > 500) {
                    // Rotate Lift Arm and drop Sample
                    liftArm.setTargetPosition(LIFT_ARM_UP);
                    lastGestureTime = curTime;
                    dropGestureState = DropYellowGestureState.RETRACT_ARM;
                }
                break;
            case RETRACT_ARM:
                if ((curTime - lastGestureTime) > 750) {
                    // Rotate Lift Arm back before dropping lift
                    liftArm.setTargetPosition(LIFT_ARM_DOWN);
                    if (fullAutonomous) {
                        //liftPowerMax = 0.4;
                        setLiftPosition(liftHeightMin);
                        if (startPosition.getX(DistanceUnit.MM) < -580) {
                            startPositionPrep = new Pose2D(DistanceUnit.MM, startPosition.getX(DistanceUnit.MM), targetPositionPrep.getY(DistanceUnit.MM), AngleUnit.DEGREES, startPosition.getHeading(AngleUnit.DEGREES));
                            dropGestureState = DropYellowGestureState.RETURN_TO_START_PREP;
                        } else {
                            dropGestureState = DropYellowGestureState.RETURN_TO_START;
                        }
                    } else {
                        dropGestureState = DropYellowGestureState.DROP_LIFT;
                    }
                    lastGestureTime = curTime;
                }
                break;
            case DROP_LIFT:
                if ((curTime - lastGestureTime) > 200) {
                    //liftPowerMax = 0.4;
                    setLiftPosition(liftHeightMin);
                    dropSampleGestureInitatied = false;
                    lastGestureTime = 0;
                }
                break;
            case RETURN_TO_START_PREP:
                if (nav.driveTo(odo.getPosition(), startPositionPrep, 1, 0)) {
                    lastGestureTime = curTime;
                    dropGestureState = DropYellowGestureState.RETURN_TO_START;
                }
                setDrivePower();
                break;
            case RETURN_TO_START:
                if (nav.driveTo(odo.getPosition(), startPosition, .5, 0)) {
                    lastGestureTime = 0;
                    gestureStep = 0;
                    dropSampleGestureInitatied = false;
                    fullAutonomous = false;
                }
                setExtensionArmPosition(ExtensionArmState.READY_TO_PICKUP);
                extensionGripper.setTargetPosition(EXT_GripperReadyToClose);
                setDrivePower();
                break;
        }

    }

    public void processDropSpecimenSampleGesture() {
        if (checkForGestureCancellation()) {
            return;
        }

        odo.update();
        double curTime = dropSampleGestureTimer.milliseconds();
        // Open Gripper - Drop block into Transfer box
        switch (dropBlueRedState) {
            case DRIVE_TO_DROP_POSITION_START:
                fullAutonomous = true;
                if (nav.driveTo(odo.getPosition(), targetPositionRight, 1, 0)) {
                    lastGestureTime = curTime;
                    extensionGripper.setTargetPosition(EXT_GripperReadyToClose);
                    if (startPosition.getX(DistanceUnit.MM) < -480) {
                        startPositionPrep = new Pose2D(DistanceUnit.MM, startPosition.getX(DistanceUnit.MM), targetPositionRight.getY(DistanceUnit.MM), AngleUnit.DEGREES, startPosition.getHeading(AngleUnit.DEGREES));
                        dropBlueRedState = dropBlueRedState.RETURN_TO_START_PREP;
                    } else {
                        dropBlueRedState = dropBlueRedState.RETURN_TO_START;
                    }
                }
                setDrivePower();

                if ((gestureStep == 0) && (curTime - lastGestureTime) < 100) {
                    extension.setTargetPosition(1);
                    extensionSpin.setTargetPosition((EXT_SpinCenter));
                    setExtensionArmPosition(ExtensionArmState.MID_HEIGHT);
                    lastGestureTime = curTime;
                    gestureStep += 1;
                }

                if ((gestureStep == 1) && (curTime - lastGestureTime) > 100) {
                    liftArm.setTargetPosition(liftArmOutOfTheWay);
                    lastGestureTime = curTime;
                    gestureStep += 1;
                }
                if ((gestureStep == 2) && (curTime - lastGestureTime) > 100) {
                    setExtensionArmPosition(ExtensionArmState.DROP_POSITION_SPECIMEN);
                    lastGestureTime = curTime;
                    gestureStep += 1;
                }
                if ((gestureStep == 3) && (curTime - lastGestureTime) > 300) {
                    extensionGripper.setTargetPosition(EXT_GripperReadyToClose);
                    lastGestureTime = curTime;
                }
                break;

            case RETURN_TO_START_PREP:
                if (nav.driveTo(odo.getPosition(), startPositionPrep, 1, 0)) {
                    lastGestureTime = curTime;
                    dropBlueRedState = dropBlueRedState.RETURN_TO_START;
                }
                setDrivePower();
                break;
            case RETURN_TO_START:
                setExtensionArmPosition(ExtensionArmState.READY_TO_PICKUP);
                extensionGripper.setTargetPosition(EXT_GripperClose);
                if (nav.driveTo(odo.getPosition(), startPosition, 1, 0)) {
                    lastGestureTime = 0;
                    gestureStep = 0;
                    dropSpecimanSampleGestureInitatied = false;
                    fullAutonomous = false;
                    extensionGripper.setTargetPosition(EXT_GripperReadyToClose);
                    liftArm.setTargetPosition(LIFT_ARM_DOWN);
                    break;
                }
                setDrivePower();
                break;
        }
    }

    public void moveExtension() {
        if (myOpMode.gamepad1.right_trigger > 0) {
            extension.setTargetPosition(extension.getPosition() - (myOpMode.gamepad1.right_trigger / 30));
        } else if (myOpMode.gamepad1.left_trigger > 0) {
            extension.setTargetPosition(extension.getPosition() + (myOpMode.gamepad1.left_trigger / 30));
        } else {
            extension.setTargetPosition(extension.getPosition());
        }
    }

    public void moveExtensionSpin() {
        double increment = .05;
        double currentPosition = extensionSpin.getPosition();
        if (myOpMode.gamepad1.right_bumper) {
            if (currentPosition > 0) {
                extensionSpin.setTargetPosition(currentPosition - increment);
            }
        } else if (myOpMode.gamepad1.left_bumper) {
            if (currentPosition < 1) {
                extensionSpin.setTargetPosition(currentPosition + increment);
            }
        }
    }

    public void drivelift() {
        if (liftEnabled || !liftEnabled) {
            liftPowerMax = 1;
            if (myOpMode.gamepad1.dpad_down) {
                setLiftPosition(liftHeightMin);
            } else if (myOpMode.gamepad1.dpad_up) {
                setLiftPosition(liftHeightMax);
            } else if (myOpMode.gamepad1.dpad_right) {
                setLiftPosition(liftHeightSpecimenDrop);
            }
        }

        // Save battery power, by not pulling down when not needed.
        if ((leftLift.getCurrentPosition()) < (liftHeightMin + 25) && (leftLift.getTargetPosition() < (liftHeightMin + 25))) {
            leftLift.setPower(0);
            rightLift.setPower(0);
        }
    }

    // Lift
    public void raiseLift() {
        liftHeight = liftHeight + 20;
        liftHeight = setLiftPosition(liftHeight);
    }

    public void lowerLift() {
        liftHeight = liftHeight - 20;
        liftHeight = setLiftPosition(liftHeight);
    }

    public int setLiftPosition(int position) {
        double liftSafetyPowerOverride = 0;
        boolean liftSafetyOverride = false;
        int liftSafetyCheck = 0;

        // Height Limits Check
        if (liftSafetyButton.getState() == false) {
            int liftHeightChange = 0;
            liftHeightChange = leftLift.getCurrentPosition() - liftHeightMin;
            liftHeightMin = leftLift.getCurrentPosition();
            liftHeightMax = liftHeightMax + liftHeightChange;
            liftHeightSpecimenDrop = liftHeightSpecimenDrop + liftHeightChange;
            if (leftLift.getCurrentPosition() > rightLift.getCurrentPosition()) {
                liftOffset = leftLift.getCurrentPosition() - rightLift.getCurrentPosition();
            } else {
                liftOffset = leftLift.getCurrentPosition() - rightLift.getCurrentPosition();
            }
        }

        if (position > liftHeightMax) position = liftHeightMax;
        else if (position < liftHeightMin) position = liftHeightMin;

        // Height Limit approaching - Power Check
        liftSafetyCheck = liftHeightMax - leftLift.getCurrentPosition();

        if (liftSafetyCheck < liftSafetyThreshold) {
            liftSafetyOverride = position > leftLift.getCurrentPosition();
            liftSafetyPowerOverride = (double) liftSafetyCheck / liftSafetyThreshold;
        } else if (leftLift.getCurrentPosition() < liftSafetyThreshold) {
            liftSafetyOverride = position < leftLift.getCurrentPosition();
            liftSafetyPowerOverride = (double) -leftLift.getCurrentPosition() / liftSafetyThreshold;
        } else {
            liftSafetyOverride = false;
        }

        if (Math.abs(liftSafetyPowerOverride) < .25) liftSafetyPowerOverride = .25;

        if (liftSafetyOverride) effectivePower = liftSafetyPowerOverride;
        else effectivePower = liftPowerMax;

        leftLift.setPower(effectivePower);
        rightLift.setPower(effectivePower);
        leftLift.setTargetPosition(position);
        rightLift.setTargetPosition(position); //- liftOffset

        return position;
    }

    public void checkCamera() {
        LLResult result = camera.getLatestResult();
        if (result != null && result.isValid()) {
            // Access color results

            botPose = result.getBotpose();

            if (!result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fr = result.getFiducialResults().get(0);
                target_X_Coor = fr.getTargetXDegrees();   // Left / Right
                target_Y_Coor = fr.getTargetYDegrees();   // Forwards / Backwards
                /*if (fr.getTargetCorners().isEmpty()) {
                    Iterator listOfLists = fr.getTargetCorners().iterator();
                    while (listOfLists.hasNext()) {
                        List<Double> list = (List<Double>) listOfLists.next();
                        myOpMode.telemetry.addData("Corners", "ORIENTATION  X: %f, Y: %f", list.get(0), list.get(1));
                    }
                }

                 */

                if (false) {
                    double positionChange = 0;
                    if ((target_Y_Coor + 11) < 0) {
                        positionChange = 0.001 * Math.abs(target_Y_Coor + 11);
                    } else {
                        positionChange = -0.001 * Math.abs(target_Y_Coor + 11);
                    }
                    extension.setTargetPosition(extension.getPosition() + positionChange);
                }
            } else {
                target_X_Coor = 0;   // Left / Right
                target_Y_Coor = 0;   // Forwards / Backwards
            }
        } else {
            botPose = null;
        }
    }

    public void driveRobot() {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        /*double axial = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = myOpMode.gamepad1.left_stick_x;
        double yaw = myOpMode.gamepad1.right_stick_x;
         */

        double axial = throttleAcceleration(-myOpMode.gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
        double lateral = throttleAcceleration(myOpMode.gamepad1.left_stick_x);
        double yaw = throttleAcceleration(myOpMode.gamepad1.right_stick_x);

        double botHeading = -odo.getHeading();

        double rotX = axial * Math.cos(-botHeading) - lateral * Math.sin(-botHeading);
        double rotY = axial * Math.sin(-botHeading) + lateral * Math.cos(-botHeading);

        driveRobotCore(-rotX, -rotY, yaw);
    }

    private void driveRobotCore(double axial, double lateral, double yaw) {
        odo.update();
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        max *= MaxPowerAdjustment;

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public double throttleAcceleration(double value) {
        double result = Math.pow(value, 2);
        if (Math.abs(value) > .2 && Math.abs(value) < .5) {
            result = .2;
        }

        if (value < 0) {
            result = result * -1;
        }
        return result;
    }

    public void setDrivePower(double leftFront, double rightFront, double leftBack,
                              double rightBack) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }

    public void setDrivePower() {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.LEFT_FRONT));
        rightFrontDrive.setPower(nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.RIGHT_FRONT));
        leftBackDrive.setPower(nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.LEFT_BACK));
        rightBackDrive.setPower(nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.RIGHT_BACK));
    }

    public void autoDriveToAprilTag() {
        checkCamera();
        if (botPose != null) {
            final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
            final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
            final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
            final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
            final double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
            final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (botPose.getPosition().x - 12); // Desired distance
            double headingError = botPose.getPosition().y;
            double yawError = botPose.getOrientation().getYaw();

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive = 0;        // Desired forward power/speed (-1 to +1)
            double strafe = 0;        // Desired strafe power/speed (-1 to +1)
            double turn = 0;        // Desired turning power/speed (-1 to +1)
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            driveRobotCore(drive, turn, strafe);
        }
    }

    public void displayTelemetry() {
        try {
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Position", data);
            if (targetPosition != null) {
                data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", targetPosition.getX(DistanceUnit.MM), targetPosition.getY(DistanceUnit.MM), targetPosition.getHeading(AngleUnit.DEGREES));
                myOpMode.telemetry.addData("Target", data);
            }
            myOpMode.telemetry.addData("Lift Current", "Left Lift: %4d Right Lift: %4d Power %1.2f", leftLift.getCurrentPosition(), rightLift.getCurrentPosition(), leftLift.getPower());
            myOpMode.telemetry.addData("Lift Target ", "Left Lift: %4d Right Lift: %4d Offset %3d", leftLift.getTargetPosition(), rightLift.getTargetPosition(), liftOffset);
            myOpMode.telemetry.addData("Lift Safety ", "Min: %4d Max: %4d Button %s Arm %1.1f", liftHeightMin, liftHeightMax, !liftSafetyButton.getState(), liftArm.getPosition());

            myOpMode.telemetry.addData("Drive Train", "LF %4d LB %4d RF %4d RB %4d Pwr:", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(), MaxPowerAdjustment);
            myOpMode.telemetry.addData("Extension", "Ext %1.1f  Arm %1.1f  Spin %1.1f  Gripper %1.1f", extension.getPosition(), extensionArm.getPosition(), extensionSpin.getPosition(), extensionGripper.getPosition());
            myOpMode.telemetry.addData("AI Camera", "Target   X: %.2f, Y: %.2f", target_X_Coor, target_Y_Coor);

            myOpMode.telemetry.addData("Angle", "Target Angle: %1.2f  Angle: %1.2f Error: %1.3f", nav.targetRadians, nav.currentRadians, nav.hError);
            myOpMode.telemetry.addData("In Bounds", "Inbounds: %s  Yaw Power: %1.3f", nav.inbounds, nav.hOutput);
            myOpMode.telemetry.addData("Gestures", "Side: %s  Initiated L: %s, Initiated R: %s ", gesturesSide, dropSampleGestureInitatied, dropSpecimanSampleGestureInitatied);

            myOpMode.telemetry.update();
            checkLiftPosition();
        } catch (Exception ex) {
            myOpMode.telemetry.addData("Error", "%s", ex.getMessage());
            myOpMode.telemetry.update();
        }
    }

}

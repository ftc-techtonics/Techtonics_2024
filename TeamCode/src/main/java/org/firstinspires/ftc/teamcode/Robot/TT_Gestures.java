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

public class TT_Gestures {

    private TT_RobotHardware robot;
    //                                                  GESTURE CONTROLS
    public boolean pickupGestureInitiated = false;
    private ElapsedTime pickupGestureTimer = new ElapsedTime();
    public boolean dropSampleGestureInitatied = false;
    public int gestureStep = 0;
    private boolean gestureAtPosition = false;
    private ElapsedTime dropSampleGestureTimer = new ElapsedTime();
    private double lastGestureTime;

    enum DropGestureState {
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

    private DropGestureState dropGestureState;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public TT_Gestures(TT_RobotHardware robotParm) {
        robot = robotParm;
    }


    public void processPickupGesture() {
        if (pickupGestureInitiated) {
            if (pickupGestureTimer.seconds() > .2) {
                robot.extensionGripper.setTargetPosition(robot.EXT_GripperClose);
            }
            if (pickupGestureTimer.seconds() > .3) {
                robot.extensionArm.setTargetPosition(robot.EXT_ArmPickupReadyHeight);
                pickupGestureInitiated = false;
            }
        }
    }

    private boolean checkForGestureCancellation() {
        if (robot.myOpMode.gamepad1.a || robot.myOpMode.gamepad1.b || robot.myOpMode.gamepad1.x || robot.myOpMode.gamepad1.y
                || robot.myOpMode.gamepad1.right_bumper || robot.myOpMode.gamepad1.left_bumper ||
                (robot.myOpMode.gamepad1.left_trigger > 0.2) || (robot.myOpMode.gamepad1.right_trigger > 0.2)) {
            return (true);
        }
        if (robot.fullAutonomous && ((Math.abs(robot.myOpMode.gamepad1.left_stick_y) > .2)
                || (Math.abs(robot.myOpMode.gamepad1.left_stick_x) > .2)
                || (Math.abs(robot.myOpMode.gamepad1.right_stick_x) > .2))) {
            return (true);
        }
        return false;
    }

    public void processDropGesture() {
        if (checkForGestureCancellation()) {
            dropSampleGestureInitatied = false;
            robot.fullAutonomous = false;
            gestureStep = 0;
            lastGestureTime = 0;
            gestureAtPosition = false;
            return;
        }

        robot.odo.update();
        double curTime = dropSampleGestureTimer.milliseconds();
        // Open Gripper - Drop block into Transfer box
        switch (dropGestureState) {
            case DRIVE_TO_DROP_POSITION_START:
                robot.fullAutonomous = true;
                if (robot.nav.driveTo(robot.odo.getPosition(), robot.targetPositionPrep, 1, 0)) {
                    lastGestureTime = curTime;
                    gestureAtPosition = true;
                }
                robot.setDrivePower();

                robot.extension.setTargetPosition(1);
                robot.extensionSpin.setTargetPosition((robot.EXT_SpinCenter));

                if (gestureStep == 0) { // && extensionGripper.getTargetPosition() == EXT_GripperClose && extensionArm.getTargetPosition() != EXT_ArmDropHeight) {
                    robot.extensionArm.setTargetPosition(robot.EXT_ArmDropHeight);
                    lastGestureTime = curTime;
                    gestureStep += 1;
                }

                if ((gestureStep == 1) && (curTime - lastGestureTime) > 1300) {
                    robot.extensionGripper.setTargetPosition(robot.EXT_GripperReadyToClose);
                    lastGestureTime = curTime;
                    gestureStep += 1;
                }

                if ((gestureStep == 2) && (curTime - lastGestureTime) > 50) {
                    robot.extensionArm.setTargetPosition(robot.EXT_ArmMidHeight);
                    lastGestureTime = curTime;
                    gestureStep = 0;
                    dropGestureState = DropGestureState.DRIVE_TO_DROP_POSITION;
                }

                break;
            case DRIVE_TO_DROP_POSITION:
                if (robot.targetPosition != null && robot.nav.driveTo(robot.odo.getPosition(), robot.targetPosition, .7, .2)) {
                    lastGestureTime = curTime;
                    dropGestureState = DropGestureState.RAISE_LIFT;
                }
                robot.setDrivePower();
                break;
            case TRANSFER_TO_LIFT_BOX:
                robot.fullAutonomous = false;
                robot.extensionGripper.setTargetPosition(robot.EXT_GripperReadyToClose);
                lastGestureTime = curTime;
                dropGestureState = DropGestureState.RAISE_LIFT;
                break;
            case RAISE_LIFT:
                if ((curTime - lastGestureTime) > 100) {
                    // Lower arm to move it out of the way of lift and bring extension in
                    robot.extensionArm.setTargetPosition(robot.EXT_ArmMidHeight);
                    robot.extensionGripper.setTargetPosition(robot.EXT_GripperClose);
                    robot.extension.setTargetPosition(1);
                    // raise lift in preparation for dropping into scoring box
                    robot.liftPowerMax = 1;
                    robot.setLiftPosition(robot.liftHeightMax);

                    lastGestureTime = curTime;
                    dropGestureState = DropGestureState.DROP_SAMPLE_IN_SCORING_BOX;
                }
                break;
            case DROP_SAMPLE_IN_SCORING_BOX:
                if ((curTime - lastGestureTime) > 500) {
                    // Rotate Lift Arm and drop Sample
                    robot.liftArm.setTargetPosition(robot.LIFT_ARM_UP);
                    robot.extensionGripper.setTargetPosition(robot.EXT_GripperReadyToClose);

                    lastGestureTime = curTime;
                    dropGestureState = DropGestureState.RETRACT_ARM;
                }
                break;
            case RETRACT_ARM:
                if ((curTime - lastGestureTime) > 750) {
                    // Rotate Lift Arm back before dropping lift
                    robot.liftArm.setTargetPosition(robot.LIFT_ARM_DOWN);
                    if (robot.fullAutonomous) {
                        robot.liftPowerMax = 0.4;
                        robot.setLiftPosition(robot.liftHeightMin);
                        if (robot.startPosition.getX(DistanceUnit.MM) < -580) {
                            robot.startPositionPrep = new Pose2D(DistanceUnit.MM, robot.startPosition.getX(DistanceUnit.MM), robot.targetPositionPrep.getY(DistanceUnit.MM), AngleUnit.DEGREES, robot.startPosition.getHeading(AngleUnit.DEGREES));
                            dropGestureState = DropGestureState.RETURN_TO_START_PREP;
                        } else {
                            dropGestureState = DropGestureState.RETURN_TO_START;
                        }
                    } else {
                        dropGestureState = DropGestureState.DROP_LIFT;
                    }
                    lastGestureTime = curTime;
                }
                break;
            case DROP_LIFT:
                if ((curTime - lastGestureTime) > 200) {
                    robot.liftPowerMax = 0.4;
                    robot.setLiftPosition(robot.liftHeightMin);
                    dropSampleGestureInitatied = false;
                    lastGestureTime = 0;
                }
                break;
            case RETURN_TO_START_PREP:
                if (robot.nav.driveTo(robot.odo.getPosition(), robot.startPositionPrep, 1, 0)) {
                    lastGestureTime = curTime;
                    dropGestureState = DropGestureState.RETURN_TO_START;
                }
                robot.setDrivePower();
                break;
            case RETURN_TO_START:
                if (robot.nav.driveTo(robot.odo.getPosition(), robot.startPosition, .5, 0)) {
                    lastGestureTime = 0;
                    gestureStep = 0;
                    dropSampleGestureInitatied = false;
                    robot.fullAutonomous = false;
                }
                robot.extensionArm.setTargetPosition(robot.EXT_ArmPickupReadyHeight);
                robot.extensionGripper.setTargetPosition(robot.EXT_GripperReadyToClose);
                robot.setDrivePower();
                break;
        }

    }
}

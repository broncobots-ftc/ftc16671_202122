/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Auto_2021_RL_Gyro", group = "ftc16671")

public class Auto_2021_RL_Gyro extends LinearOpMode {
    /** This is for encoder **/
    static final double     COUNTS_PER_MOTOR_REV    = 500 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;//0.6
    static final double     TURN_SPEED              = 0.3;//0.5
    //
    static final int oneRingMinHeight = 180;
    static final int oneRingMaxHeight = 240;

    static final int fourRingsMinHeight = 240;
    static final int fourRingsMaxHeight = 340;

    static final int SECONDS_TO_DETECT_OBJECT = 0;

    private MecanumDrive mecanumDrive = new MecanumDrive();
    private ElapsedTime runtime = new ElapsedTime();
    public MecanumDrive mDrive = new MecanumDrive();

    @Override
    public void runOpMode() throws InterruptedException {

        mecanumDrive.initVuforia(hardwareMap);
        mecanumDrive.initTfod(hardwareMap);
        mecanumDrive.init(hardwareMap);
        mecanumDrive.initCarousel_and_lift(hardwareMap);
        mecanumDrive.initServo(hardwareMap);
        mecanumDrive.initIntake(hardwareMap);
        mecanumDrive.init_IMU(hardwareMap, telemetry);
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (mecanumDrive.tfod != null) {
            mecanumDrive.tfod.activate();
            mecanumDrive.tfod.setZoom(1.0, 1.78);
            //tfod.setZoom(1, 16.0/9.0);
        }
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        long start = System.currentTimeMillis();
        long end = System.currentTimeMillis();
        int totalRings = 0;
        int level = 0;
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                if (mecanumDrive.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    float seconds = (end - start) / 1000F;

                    while (seconds <= SECONDS_TO_DETECT_OBJECT) {
                        //telemetry.addData("# seconds passed : ", seconds);
                        //telemetry.update();
                        List<Recognition> updatedRecognitions = mecanumDrive.tfod.getUpdatedRecognitions();
                        end = System.currentTimeMillis();
                        seconds = (end - start) / 1000F;
                        //telemetry.addData("# Object Detected");

                        if (updatedRecognitions != null) {

                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            telemetry.addData("# seconds passed : ", seconds);

                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("Left Side Value (%d)", i), "%.03f", recognition.getLeft());
                                telemetry.update();
                                float totalHeight = recognition.getHeight();
                                float leftSide = recognition.getLeft();
                                level = mecanumDrive.detectDuckLevel(leftSide, recognition.getLabel());
                                telemetry.addData("Level : ", level);
                                i++;
                            }
                            telemetry.update();

                        }
                    }
                    ////sleep(5000);//before it was 8000

                    // For RED1, with front against wall
                    // Identify location of shipping element/ducky
                    mecanumDrive.holder.setPosition(.73);
                    mecanumDrive.box.setPosition(.90);
                    //lift up
                    mecanumDrive.moveLiftUp(500, 0.4);
                    //
                    sleep(500);
                    mecanumDrive.box.setPosition(.66);
                    //commit does not work
                    mecanumDrive.runIntake(-1);
                    //running intake backwards to kick away element
                    //mecanumDrive.moveBackward(17.5, true, 5, 0.4, telemetry);//7, 5.6, 5.5, 5.4,5.3, 14
                    mecanumDrive.gyroDrive(DRIVE_SPEED, -61.0, 0.0, opModeIsActive(), telemetry);    // Drive back 24 inches

                    //Strafe Right
                   //mecanumDrive.strafeLeft(-10, true, 5, 0.4, telemetry);
                    sleep(250);
                    mecanumDrive.gyroTurn( TURN_SPEED, -130.0, opModeIsActive(), telemetry);         // Turn  CW to 90 Degrees
                    mecanumDrive.runIntake(0);
                    mecanumDrive.gyroDrive(DRIVE_SPEED, -15.5, -130, opModeIsActive(), telemetry);    // Drive back 24 inches
                    mecanumDrive.moveLiftUp(mecanumDrive.getLiftHeight(level), 0.8);

                    //Use dropping function
                    sleep(1500);
                    mecanumDrive.dumpAndBringbackBox();
                    //back away from hub
                    mecanumDrive.gyroDrive(DRIVE_SPEED, 4.5, -130, opModeIsActive(), telemetry);
                    //turning and driving towards to call
                    mecanumDrive.gyroTurn( TURN_SPEED, -270.0, opModeIsActive(), telemetry);
                    mecanumDrive.gyroDrive(DRIVE_SPEED, -25, -270, opModeIsActive(), telemetry);
                    //move lift down
                    mecanumDrive.moveLiftUp(500, 0.4);

                    mecanumDrive.gyroTurn( TURN_SPEED, -180.0, opModeIsActive(), telemetry);
                    mecanumDrive.gyroDrive(DRIVE_SPEED, -48, -180, opModeIsActive(), telemetry);
                    mecanumDrive.gyroTurn( TURN_SPEED, -270.0, opModeIsActive(), telemetry);
                    mecanumDrive.gyroDrive(DRIVE_SPEED, -5, -270, opModeIsActive(), telemetry);
                    mecanumDrive.runCarousel(-0.4);
                    sleep(4000);
                    mecanumDrive.strafeLeft(-8, true, 5, 0.4, telemetry);//10
                    mecanumDrive.moveBackward(2, true, 5, .4, telemetry);
                    ///mecanumDrive.gyroTurn( TURN_SPEED, -360.0, opModeIsActive(), telemetry);
                    //mecanumDrive.gyroDrive(DRIVE_SPEED, -19, -360, opModeIsActive(), telemetry);
                    //puting box back to floor to prepare for tele op
                    mecanumDrive.box.setPosition(.97);
                    sleep(1200);
                    mecanumDrive.moveLiftUp(0, 0.7); //was 0.4
                    break;

//                    mecanumDrive.rotateLeftSide(-9, true,5,0.4, telemetry);//-7.4, -8.1, -8.4
//                    //Move backward (make sure to contact the shipping hub)
//                    mecanumDrive.runIntake(0);
//                    //stop intake
//                    mecanumDrive.moveBackward(3.2, true, 5, 0.2, telemetry);//2.5, 3.1, 3
//                    mecanumDrive.moveLiftUp(mecanumDrive.getLiftHeight(level), 0.8);
//
//                    //Use dropping function
//                    sleep(1500);
//                    //wait for two seconds
//                    mecanumDrive.dumpAndBringbackBox();
//                    mecanumDrive.moveForward(2, true, 5, .4, telemetry);
//                    mecanumDrive.rotateLeftSide(1.0, true, 5, 0.4, telemetry );
//                    // Move forwaard slightly
//                    mecanumDrive.moveForward(10, true, 5, 0.4, telemetry);//5.0, 5.6, 5.7, 5.6, 5.4, 5.2, 5.3, 5.5, 5.6, 12.4
//                    //move lift down
//                    mecanumDrive.moveLiftUp(500, 0.4);
//                    //turn left 90 degrees
//                    mecanumDrive.rotateLeftSide(15.2, true, 5, 0.4, telemetry);//3, 7.5, 7.3, 14.8, 15.5, 15, 14, 14.3, 14.9
//                    //Go foward and park in the warehouse
//                    mecanumDrive.strafeRight(20, true, 5, 0.4, telemetry);//12, 18, 7.2, 15, 16, 18, 20,19
//                    mecanumDrive.runCarousel(-0.4);
//
//                    sleep(5000);
//                    mecanumDrive.strafeLeft(-8, true, 5, 0.4, telemetry);//10
//                    mecanumDrive.moveBackward(3, true, 5, 0.4, telemetry);
//                    //puting box back to floor to prepare for tele op
//                    mecanumDrive.box.setPosition(1);
//                    sleep(1200);
//                    mecanumDrive.moveLiftUp(0, 0.7); //was 0.4
//                    break;
                    //to commit

                    //}
                }
            }
            if (mecanumDrive.tfod != null) {
                mecanumDrive.tfod.shutdown();
            }
        }
    }

}

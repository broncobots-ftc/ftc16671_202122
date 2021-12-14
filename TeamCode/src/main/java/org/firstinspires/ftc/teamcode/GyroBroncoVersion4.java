
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "GyroBroncoVersion4")
public class GyroBroncoVersion4 extends LinearOpMode {

    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor back_left_motor;
    private DcMotor back_right_motor;
    private BNO055IMU imu;

    static final double     COUNTS_PER_MOTOR_REV    = ((((1+(46/17))) * (1+(46/11))) * 28);    // eg:
    static final double     DRIVE_GEAR_REDUCTION    = 1.25 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = (96/25.4) ;     // 96mm For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.3;

    static final double     HEADING_THRESHOLD       = 5 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters IMU_Parameters;
        ElapsedTime ElapsedTime2;
        double Left_Power;
        double Right_Power;
        float Yaw_Angle = 0 ;

        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse direction of one motor so robot moves
        // forward rather than spinning in place.
        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);

//        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
//        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Create an IMU parameters object.
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);
        // Report the initialization to the Driver Station.
        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();
        // Wait one second to ensure the IMU is ready.
        sleep(1000);
        // Loop until IMU has been calibrated.
        while (!IMU_Calibrated()) {
            telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
            telemetry.update();
            // Wait one second before checking calibration
            // status again.
            sleep(1000);
        }
        // Report calibration complete to Driver Station.
        telemetry.addData("Status", "Calibration Complete");
        telemetry.addData("Action needed:", "Please press the start triangle");
        telemetry.update();
        // Wait for Start to be pressed on Driver Station.
        waitForStart();
        // Create a timer object with millisecond
        // resolution and save in ElapsedTime variable.
        ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // Initialize motor power variables to 30%.
        Left_Power = 0.3;
        Right_Power = 0.3;
        // Set motor powers to the variable values.
        front_left_motor.setPower(Left_Power);
        front_right_motor.setPower(Right_Power);
        back_left_motor.setPower(Left_Power);
        back_right_motor.setPower(Right_Power);

        gyroDrive(DRIVE_SPEED, -58.0, 0.0);    // Drive back 24 inches
        sleep(250);
        gyroTurn( TURN_SPEED, -130.0);         // Turn  CW to 90 Degrees

        gyroDrive(DRIVE_SPEED, -12, -130);    // Drive back 24 inches
        sleep(1000);
        gyroDrive(DRIVE_SPEED, 3, -130);
        gyroTurn( TURN_SPEED, -220.0);
        gyroDrive(DRIVE_SPEED, -50, -220);
        gyroTurn( TURN_SPEED, -270.0);

/*
//
//         Move robot forward for 2 seconds or until stop
//         is pressed on Driver Station.

        while (!(ElapsedTime2.milliseconds() >= 2000 || isStopRequested())) {
            // Save gyro's yaw angle
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw angle", Yaw_Angle);
            // If the robot is moving straight ahead the
            // yaw value will be close to zero. If it's not, we
            // need to adjust the motor powers to adjust heading.
            // If robot yaws right or left by 5 or more,
            // adjust motor power variables to compensation.
            if (Yaw_Angle < -5) {
                // Turn left
                Left_Power = 0.25;
                Right_Power = 0.35;
            } else if (Yaw_Angle > 5) {
                // Turn right.
                Left_Power = 0.35;
                Right_Power = 0.25;
            } else {
                // Continue straight
                Left_Power = 0.3;
                Right_Power = 0.3;
            }
            // Report the new power levels to the Driver Station.
            telemetry.addData("Left Motor Power", Left_Power);
            telemetry.addData("Right Motor Power", Right_Power);
            // Update the motors to the new power levels.
            front_left_motor.setPower(Left_Power);
            front_right_motor.setPower(Right_Power);
            back_left_motor.setPower(Left_Power);
            back_right_motor.setPower(Right_Power);
            telemetry.update();
            // Wait 1/5 second before checking again.
            sleep(200);
        }
        // Now let's execute a right turn using power
        // levels that will cause a turn in place.
        front_left_motor.setPower(0.2);
        front_right_motor.setPower(-0.2);
        back_left_motor.setPower(0.2);
        back_right_motor.setPower(-0.2);
        // Continue until robot yaws right by 90 degrees
        // or stop is pressed on Driver Station.
        while (!(Yaw_Angle <= -90 || isStopRequested())) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);

 */
    }

    /**
     * Function that becomes true when gyro is calibrated and
     * reports calibration status to Driver Station in the meantime.
     */
    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", imu.getSystemStatus().toString());
        return imu.isGyroCalibrated();
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = front_left_motor.getCurrentPosition() + moveCounts;
            newRightTarget = front_right_motor.getCurrentPosition() + moveCounts;



            // Set Target and Turn On RUN_TO_POSITION
            front_left_motor.setTargetPosition(newLeftTarget);
            front_right_motor.setTargetPosition(newRightTarget);
            back_left_motor.setTargetPosition(newLeftTarget);
            back_right_motor.setTargetPosition(newRightTarget);

            front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            front_left_motor.setPower(speed);
            front_right_motor.setPower(speed);
            back_left_motor.setPower(speed);
            back_right_motor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (front_left_motor.isBusy() && front_right_motor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

//                robot.leftDrive.setPower(leftSpeed);
//                robot.rightDrive.setPower(rightSpeed);
                front_left_motor.setPower(leftSpeed);
                front_right_motor.setPower(rightSpeed);
                back_left_motor.setPower(leftSpeed);
                back_right_motor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      front_left_motor.getCurrentPosition(),
                        front_right_motor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
//            robot.leftDrive.setPower(0);
//            robot.rightDrive.setPower(0);
            front_left_motor.setPower(0);
            front_right_motor.setPower(0);
            back_left_motor.setPower(0);
            back_right_motor.setPower(0);

            // Turn off RUN_TO_POSITION
//            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }




    /**
                             *  Method to spin on central axis to point in a new direction.
                             *  Move will stop if either of these conditions occur:
                             *  1) Move gets to the heading (angle)
                             *  2) Driver stops the opmode running.
                             *
                             * @param speed Desired speed of turn.
                             * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
                             *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
                             *                   If a relative angle is required, add/subtract from current heading.
                             */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        front_left_motor.setPower(leftSpeed);
        front_right_motor.setPower(rightSpeed);
        back_left_motor.setPower(leftSpeed);
        back_right_motor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - imu.getAngularOrientation().getIntegratedZValue();
        robotError = targetAngle-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}


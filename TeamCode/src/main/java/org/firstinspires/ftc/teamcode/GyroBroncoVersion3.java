
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "GyroBroncoVersion3")
public class GyroBroncoVersion3 extends LinearOpMode {

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

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


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
        // Move robot forward for 2 seconds or until stop
        // is pressed on Driver Station.

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
}


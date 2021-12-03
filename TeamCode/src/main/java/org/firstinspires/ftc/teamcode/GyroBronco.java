//Autonomous mode Guga 12/2/2021

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "GyroBronco")
public class GyroBronco extends LinearOpMode {

    private DcMotor front_left_motor;
    private DcMotor back_left_motor;
    private DcMotor front_right_motor;
    private DcMotor back_right_motor;
    private BNO055IMU imu;


    float xOrientation;
    //float encoderResolution= =((((1+(46/17))) * (1+(46/11))) * 28);
    //float wheelRevolution = (2*3.14159265359*95)/25.4 ;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");


        // Reveser direction of left motor
        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Place into braking mode so robot stops abruptly
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Init the IMU
        // Current X orientation to zero- X is upward.  The robot direction will be zero until we do something to change it.
        init_IMU();
        // Robots direction will be zero until we do something
        // Wait for driver to press start button
        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("X axis angle ", getXAxisOrientation());
            telemetry.addData("Y axis angle ", getYAxisOrientation());
            telemetry.addData("Z axis angle ", getZAxisOrientation());
            telemetry.update();
            sleep(2000);
            // To travel 24 in, we need = 2.02 x 537.7 pulses
            Move_to_Position(2.02 * 537.7);
            telemetry.addData("after first move", " ");
            telemetry.addData("X axis angle ", getXAxisOrientation());
            telemetry.addData("Y axis angle ", getYAxisOrientation());
            telemetry.addData("Z axis angle ", getZAxisOrientation());
            telemetry.update();
            sleep(2000);
            // turn CW unti you reach 90
            rotateCW(90);
            telemetry.addData("after rotate", " ");
            telemetry.addData("X axis angle ", getXAxisOrientation());
            telemetry.addData("Y axis angle ", getYAxisOrientation());
            telemetry.addData("Z axis angle ", getZAxisOrientation());
            telemetry.update();
            sleep(2000);
            //rotateCW(90);
            // To travel 24 in, we need = 2.02 x 537.7 pulses
            Move_to_Position(2.02 * 537.7);
            telemetry.addData("last move", " ");
            telemetry.addData("X axis angle ", getXAxisOrientation());
            telemetry.addData("Y axis angle ", getYAxisOrientation());
            telemetry.addData("Z axis angle ", getZAxisOrientation());
            telemetry.update();


        }
    }

    /**
     * initialize imu...
     */
    private void init_IMU() {
        BNO055IMU.Parameters IMUparameters;

        // Create a new IMU parameter object
        IMUparameters = new BNO055IMU.Parameters();
        // Set the imu mode to imu so automatically calibrate itself
        IMUparameters.mode = BNO055IMU.SensorMode.IMU;
        // Use degrees as angle unit
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // use meters per seconds^2 for units of acceleration
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Warn driver it may take several seconds
        telemetry.addData("Status", "Init IMU... Please Wait");
        telemetry.update();
        // Initialize IMU using these parameters
        imu.initialize(IMUparameters);
        // Tell Drive the init is done
        telemetry.addData("Status", "IMU initialized");
        telemetry.update();
    }

    /**
     * Move to a position.  We need to get the correct pulse and wheel diameter.  ...
     */
    private void Move_to_Position(double TargetPosition) {
        // Reset the encoders
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Put the motor in encoder mode
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Turn on moters using moderate power
        front_left_motor.setPower(0.3);
        front_right_motor.setPower(0.3);
        back_left_motor.setPower(0.3);
        back_right_motor.setPower(0.3);
        // Loop until the motor reaches its target position
        while (front_left_motor.getCurrentPosition() < TargetPosition) {
            // Nothing while the robot moves forward
        }
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        // sleep quarter of seconds to let the robot sleep
        sleep(250);
    }

    /**
     * Get the orientation of the axis.  Here X is up vertical.
     */
    private float getXAxisOrientation() {
        Orientation Angles;

        // Get X axis orientation of the IMU.  Typically Z is the Yaw, but here in our robot, we have X axis is vertically Upward
        // we stand behind the robot, the rev stay flat, first at the bottom, right sensor, left motor, ZYX, but here for us it is XYZ
        Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return Angles.firstAngle;
    }

    private float getYAxisOrientation() {
        Orientation Angles;

        // Get X axis orientation of the IMU.  Typically Z is the Yaw, but here in our robot, we have X axis is vertically Upward
        // we stand behind the robot, the rev stay flat, first at the bottom, right sensor, left motor, ZYX, but here for us it is XYZ
        Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return Angles.secondAngle;
    }

    private float getZAxisOrientation() {
        Orientation Angles;

        // Get X axis orientation of the IMU.  Typically Z is the Yaw, but here in our robot, we have X axis is vertically Upward
        // we stand behind the robot, the rev stay flat, first at the bottom, right sensor, left motor, ZYX, but here for us it is XYZ
        Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return Angles.thirdAngle;
    }

    /**
     * This function is to rotate clockwise..
     */
    private void rotateCCW(int targetOrientationAngle) {
        // Rotate in CCW direction
        // Assume we have not turned more than 180 degree
        // Get initial orientation about the X axis in our robot which is verticle upward
        xOrientation = getXAxisOrientation();
        // Set power to robot rotate in CCW direction
        front_left_motor.setPower(-0.8);
        back_left_motor.setPower(-0.8);
        front_right_motor.setPower(0.8);
        back_right_motor.setPower(0.8);
        // loop until we reach the target orientation
        while (xOrientation < targetOrientationAngle) {
            // Update current orientation about X
            xOrientation = getXAxisOrientation();
        }
        // stop the motors
        front_left_motor.setPower(0);
        back_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_right_motor.setPower(0);
        // wait a moment for the robot to stop
        sleep(250);
    }

    /**
     * Counterclockwise rotation.
     */
    private void rotateCW(float targetOrientationAngle) {
        // Rotate in CW direction
        // Assume we have not turned more than 180 degree
        // Get initial orientation about the X axis in our robot which is verticle upward
        xOrientation = getXAxisOrientation();
        if(Math.abs(xOrientation) != 0){
            targetOrientationAngle = targetOrientationAngle - xOrientation;
        }
        // Set power to robot rotate in CCW direction
        front_left_motor.setPower(0.8);
        back_left_motor.setPower(0.8);
        front_right_motor.setPower(-0.8);
        back_right_motor.setPower(-0.8);
        // loop until we reach the target orientation
        while (xOrientation < targetOrientationAngle) {
            // Update current orientation about X
            xOrientation = getXAxisOrientation();
        }
        // stop the motors
        front_left_motor.setPower(0);
        back_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_right_motor.setPower(0);
        // wait a moment for the robot to stop
        sleep(250);
    }
}

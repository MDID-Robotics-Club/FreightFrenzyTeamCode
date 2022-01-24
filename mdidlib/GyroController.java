package org.firstinspires.ftc.teamcode.mdidlib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroController {

    public BNO055IMU gyro;
    public BNO055IMU.Parameters parameters;
    double p;

    Orientation angles;

    public GyroController(BNO055IMU defaultGyro) {
        gyro = defaultGyro;
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(parameters);
    }

    public GyroController(ModernRoboticsI2cGyro I2cGyro) {

    }

    public void setPConstant(double kP) {
        p = kP;
    }

    public Orientation getAngle() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles;
    }

    public double[] maintainEncoderDriveAngle(double referenceAngle, double stateAngle) {
        double leftPower = 0;
        double rightPower = 0;
        double[] output = {leftPower, rightPower};
        return output;

    }

    public double[] turnEncoderTurnToAngle(double referenceAngle, double stateAngle) {
        double leftPower = 0;
        double rightPower = 0;
        double[] output = {leftPower, rightPower};
        return output;
    }
}

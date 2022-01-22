package org.firstinspires.ftc.teamcode.mdidlib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

public class GyroController {

    public BNO055IMU gyro;
    public BNO055IMU.Parameters parameters;
    double p;

    public GyroController(BNO055IMU defaultGyro) {
        gyro = defaultGyro;
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.initialize(parameters);
    }

    public GyroController(ModernRoboticsI2cGyro I2cGyro) {

    }

    public void setPConstant(double kP) {
        p = kP;
    }

    public double[] maintainEncoderDriveAngle(double referenceAngle, double stateAngle) {

    }

    public double[] turnEncoderTurnToAngle(double referenceAngle, double stateAngle) {

    }
}

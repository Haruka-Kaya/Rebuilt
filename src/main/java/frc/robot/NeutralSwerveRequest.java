package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

/** Replaces previous drive and steer outputs with the configured neutral mode. */
final class NeutralSwerveRequest implements SwerveRequest {
    // The odometry thread resends these requests. Do not leave a periodic sender
    // running after another swerve request takes control.
    private final NeutralOut driveNeutral = new NeutralOut().withUpdateFreqHz(0);
    private final NeutralOut steerNeutral = new NeutralOut().withUpdateFreqHz(0);

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modules) {
        StatusCode result = StatusCode.OK;
        for (var module : modules) {
            StatusCode driveStatus = neutralize(module.getDriveMotor(), driveNeutral);
            StatusCode steerStatus = neutralize(module.getSteerMotor(), steerNeutral);
            if (result.isOK() && !driveStatus.isOK()) result = driveStatus;
            if (result.isOK() && !steerStatus.isOK()) result = steerStatus;
        }
        return result;
    }

    private static StatusCode neutralize(CommonTalon motor, NeutralOut request) {
        try {
            StatusCode status = motor.setControl(request);
            return status == null ? StatusCode.GeneralError : status;
        } catch (RuntimeException error) {
            // One failed controller must not prevent attempts to neutralize the
            // remaining controllers. A later odometry iteration retries them.
            return StatusCode.GeneralError;
        }
    }
}

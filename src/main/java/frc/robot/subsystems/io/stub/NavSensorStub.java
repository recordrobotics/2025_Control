package frc.robot.subsystems.io.stub;

import frc.robot.subsystems.io.NavSensorIO;

public class NavSensorStub implements NavSensorIO {

    @Override
    public void reset() {
        /* stub */
    }

    @Override
    public void resetDisplacement() {
        /* stub */
    }

    @Override
    public double getAngle() {
        return 0;
    }

    @Override
    public double getWorldLinearAccelX() {
        return 0;
    }

    @Override
    public double getWorldLinearAccelY() {
        return 0;
    }

    @Override
    public boolean isConnected() {
        return false;
    }

    @Override
    public void close() throws Exception {
        /* stub */
    }

    @Override
    public void simulationPeriodic() {
        /* stub */
    }
}

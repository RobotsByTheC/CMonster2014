/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2012. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.parsing.IUtility;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.util.BoundaryException;

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes care of
 * the integral calculations, as well as writing the given PIDOutput
 */
public class SynchronousPIDController implements IUtility, LiveWindowSendable {

    private double p;			// factor for "proportional" control
    private double i;			// factor for "integral" control
    private double d;			// factor for "derivative" control
    private double f;                 // factor for feedforward term
    private double maximumOutput = 1.0;	// |maximum output|
    private double minimumOutput = -1.0;	// |minimum output|
    private double maximumInput = 0.0;		// maximum input - limit setpoint to this
    private double minimumInput = 0.0;		// minimum input - limit setpoint to this
    private boolean continuous = false;	// do the endpoints wrap around? eg. Absolute encoder
    private double prevError = 0.0;	// the prior sensor input (used to compute velocity)
    private double totalError = 0.0; //the sum of the errors for use in the integral calc
    private Tolerance tolerance;	//the tolerance object used to check if on target
    private double setpoint = 0.0;
    private double error = 0.0;
    private double lastTime = Utility.getFPGATime() / 1000000.0;

    /**
     * Tolerance is the type of tolerance used to specify if the PID controller
     * is on target. The various implementations of this class such as
     * PercentageTolerance and AbsoluteTolerance specify types of tolerance
     * specifications to use.
     */
    public interface Tolerance {

        public boolean onTarget(double value);
    }

    public class PercentageTolerance implements Tolerance {

        double percentage;

        PercentageTolerance(double value) {
            percentage = value;
        }

        public boolean onTarget(double value) {
            return (Math.abs(value) < percentage / 100
                    * (maximumInput - minimumInput));
        }
    }

    public class AbsoluteTolerance implements Tolerance {

        double value;

        AbsoluteTolerance(double value) {
            this.value = value;
        }

        public boolean onTarget(double error) {
            return Math.abs(error) < value;
        }
    }

    public class NullTolerance implements Tolerance {

        public boolean onTarget(double value) {
            throw new RuntimeException("No tolerance value set when using PIDController.onTarget()");
        }
    }

    /**
     * Allocate a PID object with the given constants for P, I, D, and F
     *
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     * @param Kf the feed forward term
     */
    public SynchronousPIDController(double Kp, double Ki, double Kd, double Kf) {
        p = Kp;
        i = Ki;
        d = Kd;
        f = Kf;

        tolerance = new NullTolerance();
    }

    /**
     * Allocate a PID object with the given constants for P, I, D and period
     *
     * @param Kp
     * @param Ki
     * @param Kd
     */
    public SynchronousPIDController(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0.0);
    }

    /**
     * Calculates the output based on an input.
     *
     * @param input the input for the PID loop
     * @return the output for the PID loop
     */
    public double calculate(double input) {
        double result = 0;
        error = setpoint - input;

        if (continuous) {
            if (Math.abs(error)
                    > (maximumInput - minimumInput) / 2) {
                if (error > 0) {
                    error = error - maximumInput + minimumInput;
                } else {
                    error = error
                            + maximumInput - minimumInput;
                }
            }
        }
        if (!tolerance.onTarget(error)) {
            double currTime = Utility.getFPGATime() / 1000000.0; // Current time in seconds
            double elapsedTime = currTime - lastTime;
            if (i != 0) {
                double potentialIGain = (totalError + error) * i;
                if (potentialIGain < maximumOutput) {
                    if (potentialIGain > minimumOutput) {
                        totalError += error;
                    } else {
                        totalError = minimumOutput / i;
                    }
                } else {
                    totalError = maximumOutput / i;
                }
            }

            result = p * error // P
                    + i * totalError // I
                    + d * (error - prevError) * elapsedTime // D
                    + setpoint * f; // F
            prevError = error;
            lastTime = currTime;

            if (result > maximumOutput) {
                result = maximumOutput;
            } else if (result < minimumOutput) {
                result = minimumOutput;
            }

        }
        return result;
    }

    /**
     * Set the PID Controller gain parameters. Set the proportional, integral,
     * and differential coefficients.
     *
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public synchronized void setPID(double p, double i, double d) {
        setPID(p, i, d, f);
    }

    /**
     * Set the PID Controller gain parameters. Set the proportional, integral,
     * and differential coefficients.
     *
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     * @param f Feed forward coefficient
     */
    public synchronized void setPID(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;

        if (table != null) {
            table.putNumber("p", p);
            table.putNumber("i", i);
            table.putNumber("d", d);
            table.putNumber("f", f);
        }
    }

    /**
     * Get the Proportional coefficient
     *
     * @return proportional coefficient
     */
    public double getP() {
        return p;
    }

    /**
     * Get the Integral coefficient
     *
     * @return integral coefficient
     */
    public double getI() {
        return i;
    }

    /**
     * Get the Differential coefficient
     *
     * @return differential coefficient
     */
    public synchronized double getD() {
        return d;
    }

    /**
     * Get the Feed forward coefficient
     *
     * @return feed forward coefficient
     */
    public synchronized double getF() {
        return f;
    }

    /**
     * Set the PID controller to consider the input to be continuous, Rather
     * then using the max and min in as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the
     * setpoint.
     *
     * @param continuous Set to true turns on continuous, false turns off
     * continuous
     */
    public synchronized void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    /**
     * Set the PID controller to consider the input to be continuous, Rather
     * then using the max and min in as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the
     * setpoint.
     */
    public synchronized void setContinuous() {
        this.setContinuous(true);
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum percentage expected from the input
     * @param maximumInput the maximum percentage expected from the output
     */
    public synchronized void setInputRange(double minimumInput, double maximumInput) {
        if (minimumInput > maximumInput) {
            throw new BoundaryException("Lower bound is greater than upper bound");
        }
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
        setSetpoint(setpoint);
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum percentage to write to the output
     * @param maximumOutput the maximum percentage to write to the output
     */
    public synchronized void setOutputRange(double minimumOutput, double maximumOutput) {
        if (minimumOutput > maximumOutput) {
            throw new BoundaryException("Lower bound is greater than upper bound");
        }
        this.minimumOutput = minimumOutput;
        this.maximumOutput = maximumOutput;
    }

    /**
     * Set the setpoint for the PIDController
     *
     * @param setpoint the desired setpoint
     */
    public synchronized void setSetpoint(double setpoint) {
        if (maximumInput > minimumInput) {
            if (setpoint > maximumInput) {
                this.setpoint = maximumInput;
            } else if (setpoint < minimumInput) {
                this.setpoint = minimumInput;
            } else {
                this.setpoint = setpoint;
            }
        } else {
            this.setpoint = setpoint;
        }

        if (table != null) {
            table.putNumber("setpoint", this.setpoint);
        }
    }

    /**
     * Returns the current setpoint of the PIDController
     *
     * @return the current setpoint
     */
    public synchronized double getSetpoint() {
        return setpoint;
    }

    /**
     * Set the PID tolerance using a Tolerance object. Tolerance can be
     * specified as a percentage of the range or as an absolute value. The
     * Tolerance object encapsulates those options in an object. Use it by
     * creating the type of tolerance that you want to use: setTolerance(new
     * PIDController.AbsoluteTolerance(0.1))
     *
     * @param tolerance a tolerance object of the right type, e.g.
     * PercentTolerance or AbsoluteTolerance
     */
    private void setTolerance(Tolerance tolerance) {
        this.tolerance = tolerance;
    }

    private Tolerance getTolerance() {
        return tolerance;
    }

    /**
     * Set the absolute error which is considered tolerable for use with
     * OnTarget.
     *
     * @param absvalue error which is tolerable in the units of the input object
     */
    public synchronized void setAbsoluteTolerance(double absvalue) {
        tolerance = new AbsoluteTolerance(absvalue);
    }

    /**
     * Set the percentage error which is considered tolerable for use with
     * OnTarget. (Input of 15.0 = 15 percent)
     *
     * @param percentage error which is tolerable
     */
    public synchronized void setPercentTolerance(double percentage) {
        tolerance = new PercentageTolerance(percentage);
    }

    /**
     * Reset the previous error, the integral term, and disable the controller.
     */
    public synchronized void reset() {
        prevError = 0;
        totalError = 0;
        lastTime = Utility.getFPGATime() / 1000000.0;
    }

    public String getSmartDashboardType() {
        return "PIDController";
    }

    private final ITableListener listener = new ITableListener() {
        public void valueChanged(ITable table, String key, Object value, boolean isNew) {
            if (key.equals("p") || key.equals("i") || key.equals("d") || key.equals("f")) {
                if (p != table.getNumber("p", 0.0) || i != table.getNumber("i", 0.0)
                        || d != table.getNumber("d", 0.0) || f != table.getNumber("f", 0.0)) {
                    setPID(table.getNumber("p", 0.0), table.getNumber("i", 0.0), table.getNumber("d", 0.0), table.getNumber("f", 0.0));
                }
            } else if (key.equals("setpoint")) {
                if (setpoint != ((Double) value).doubleValue()) {
                    setSetpoint(((Double) value).doubleValue());
                }
            }
        }
    };
    private ITable table;

    public void initTable(ITable table) {
        if (this.table != null) {
            this.table.removeTableListener(listener);
        }
        this.table = table;
        if (table != null) {
            table.putNumber("p", getP());
            table.putNumber("i", getI());
            table.putNumber("d", getD());
            table.putNumber("f", getF());
            table.putNumber("setpoint", getSetpoint());
            table.addTableListener(listener, false);
        }
    }

    /**
     * {@inheritDoc}
     */
    public ITable getTable() {
        return table;
    }

    /**
     * {@inheritDoc}
     */
    public void updateTable() {
    }

    /**
     * {@inheritDoc}
     */
    public void startLiveWindowMode() {
    }

    /**
     * {@inheritDoc}
     */
    public void stopLiveWindowMode() {
    }
}

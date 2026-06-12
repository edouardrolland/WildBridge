package dji.sampleV5.moduleaircraft.controller
import kotlin.math.max
import kotlin.math.min

/**
 * This class implements the PID functionality with anti-windup
 */
class PID(
        private val kp: Double,
        private val ki: Double,
        private val kd: Double,
        private val dt: Double,
        private val outputLimits: Pair<Double?, Double?>
) {
    private var integral = 0.0
    private var previousError = 0.0
    private var hasPreviousError = false

    /**
     * Clear accumulated state (integral + derivative history). Call this when the
     * setpoint changes discontinuously — e.g. a hot-swapped waypoint — so the sudden
     * jump in error does not produce an integral/derivative kick.
     */
    fun reset() {
        integral = 0.0
        previousError = 0.0
        hasPreviousError = false
    }

    /** Update using the fixed nominal timestep supplied at construction. */
    fun update(error: Double): Double = update(error, dt)

    /**
     * Update using an explicitly measured timestep [dtSec]. Preferred for loops driven
     * by a Handler, whose actual cadence drifts under main-thread load: feeding the real
     * elapsed time keeps the integral/derivative terms correct instead of assuming [dt].
     */
    fun update(error: Double, dtSec: Double): Double {
        // Proportional term
        val p = kp * error

        // Derivative term. Skipped on the very first sample after construction or reset(),
        // otherwise a cold-start setpoint produces a spurious derivative spike.
        val derivative = if (hasPreviousError && dtSec != 0.0) (error - previousError) / dtSec else 0.0
        val d = kd * derivative
        previousError = error
        hasPreviousError = true

        // Integral term with anti-windup
        integral += error * dtSec

        // Calculate output before applying integral term
        var output = p + d

        // Apply output limits to check for saturation
        val (minOutput, maxOutput) = outputLimits
        val outputUnclamped = output + ki * integral

        // Anti-windup: Only update integral if output is not saturated
        val i = if ((minOutput == null || outputUnclamped > minOutput) &&
                (maxOutput == null || outputUnclamped < maxOutput)) {
            ki * integral
        } else {
            0.0
        }

        // PID output before limits
        output += i

        // Apply output limits
        if (minOutput != null) output = max(minOutput, output)
        if (maxOutput != null) output = min(maxOutput, output)

        return output
    }
}

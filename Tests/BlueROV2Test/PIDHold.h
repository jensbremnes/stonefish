/*
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  PIDHold.h
//  Stonefish
//
//  The small amount of control theory the BlueROV2 autopilot needs: a first order low pass,
//  a derivative that copes with a sensor slower than the loop around it, and a PID.
//
//  Stonefish has no controller infrastructure of its own - there is no controllers/ directory
//  anywhere in the library, and the only feedback code in the tree is sf::MechanicalPI, which
//  lives inside the thruster rotor loop, and the P loop inside sf::Servo. The integrator here
//  is clamped the same way MechanicalPI clamps its own.
//
//  Header only on purpose: UnderIceTest compiles BlueROV2Test/BlueROV2TestApp.cpp straight into
//  its own target, so a new .cpp would have to be added to two places in Tests/CMakeLists.txt.
//

#ifndef __Stonefish__PIDHold__
#define __Stonefish__PIDHold__

#include <StonefishCommon.h>

#include <cmath>

namespace brov2
{
    //! Wraps an angle into [-pi, pi]. Heading errors are useless without this: a turn from
    //! 179 to -179 degrees is 2 degrees to starboard, not 358 to port.
    inline sf::Scalar WrapPi(sf::Scalar a)
    {
        while(a > SIMD_PI)  a -= SIMD_2_PI;
        while(a < -SIMD_PI) a += SIMD_2_PI;
        return a;
    }

    //! First order low pass, used on the measured rates that feed the D terms.
    class LowPass
    {
    public:
        explicit LowPass(sf::Scalar tau) : tau_(tau), y_(0), init_(false) {}

        void Reset() { y_ = sf::Scalar(0); init_ = false; }

        sf::Scalar Update(sf::Scalar x, sf::Scalar dt)
        {
            if(!init_) //Start on the signal, not on zero, so there is no settling transient
            {
                init_ = true;
                y_ = x;
                return y_;
            }
            //Exact discretisation of the lag rather than the Euler form, so a long frame
            //cannot drive the filter unstable
            const sf::Scalar a = tau_ > sf::Scalar(0)
                               ? sf::Scalar(1) - sf::Scalar(std::exp(-(double)dt / (double)tau_))
                               : sf::Scalar(1);
            y_ += a * (x - y_);
            return y_;
        }

        sf::Scalar getValue() const { return y_; }

    private:
        sf::Scalar tau_;
        sf::Scalar y_;
        bool init_;
    };

    //! Filtered derivative of a signal sampled more slowly than this is called.
    //!
    //! The pressure sensor runs at 10 Hz and the loop reading it at frame rate, so a plain
    //! (x - last)/dt sees five or six frames of no change for every one that moves and reports
    //! a rate that is mostly zero. This accumulates dt until the value actually changes and
    //! divides by the accumulated interval instead.
    //!
    //! The equality test is exact on purpose: between sensor updates getLastValue returns the
    //! identical stored Scalar, and every real update carries measurement noise, so two
    //! successive samples are never bit-identical.
    class Derivative
    {
    public:
        explicit Derivative(sf::Scalar tau) : lp_(tau), last_(0), acc_(0), init_(false) {}

        void Reset() { lp_.Reset(); last_ = acc_ = sf::Scalar(0); init_ = false; }

        sf::Scalar Update(sf::Scalar x, sf::Scalar dt)
        {
            if(!init_)
            {
                init_ = true;
                last_ = x;
                acc_ = sf::Scalar(0);
                return sf::Scalar(0);
            }

            acc_ += dt;
            if(x == last_) //No new sample yet - keep the last estimate rather than decaying it
                return lp_.getValue();

            const sf::Scalar interval = acc_;
            const sf::Scalar raw = interval > sf::Scalar(0) ? (x - last_) / interval : sf::Scalar(0);
            last_ = x;
            acc_ = sf::Scalar(0);
            return lp_.Update(raw, interval);
        }

        sf::Scalar getValue() const { return lp_.getValue(); }

    private:
        LowPass lp_;
        sf::Scalar last_;
        sf::Scalar acc_;
        bool init_;
    };

    //! PID whose D term is taken on a SUPPLIED measured rate rather than on the error.
    //!
    //! Both matter here. The rates come straight off the IMU, so there is nothing to
    //! differentiate; and in this autopilot the pilot moves the setpoint whenever a key is
    //! held, which a D term on the error would answer with a kick on every keystroke.
    class PIDHold
    {
    public:
        //! iBand is the largest error the integrator will accumulate. Outside it the term is
        //! frozen, so a big commanded move does not wind it up on the way.
        PIDHold(sf::Scalar kp, sf::Scalar ki, sf::Scalar kd, sf::Scalar iBand, sf::Scalar iLimit,
                sf::Scalar outLimit)
            : kp_(kp), ki_(ki), kd_(kd), iBand_(iBand), iLimit_(iLimit), outLimit_(outLimit),
              iTerm_(0), out_(0) {}

        //! Seeds the integrator, so a loop can be engaged already carrying a known trim.
        void Reset(sf::Scalar iSeed = sf::Scalar(0))
        {
            iTerm_ = btClamped(iSeed, -iLimit_, iLimit_);
            out_ = sf::Scalar(0);
        }

        //! \param error setpoint - measurement
        //! \param measRate d(measurement)/dt, already filtered
        sf::Scalar Update(sf::Scalar error, sf::Scalar measRate, sf::Scalar dt)
        {
            out_ = btClamped(kp_ * error + iTerm_ - kd_ * measRate, -outLimit_, outLimit_);

            //Two guards on the integrator, and measurement wanted both.
            //
            //Conditional anti-windup: integrate only when that would not drive an already
            //saturated output further into its limit.
            //
            //Integration band: integrate only near the setpoint. The integrator here exists to
            //trim a standing bias - the vehicle's +2.0 N of net buoyancy, mostly - not to chase
            //a commanded move. Without the band a 2 m depth step wound it to its limit during
            //the six second transit and the vehicle sailed 0.5 m past.
            const bool pushingIntoLimit = (out_ >= outLimit_ && error > sf::Scalar(0))
                                       || (out_ <= -outLimit_ && error < sf::Scalar(0));
            const bool nearSetpoint = btFabs(error) <= iBand_;
            if(dt > sf::Scalar(0) && nearSetpoint && !pushingIntoLimit)
                iTerm_ = btClamped(iTerm_ + ki_ * error * dt, -iLimit_, iLimit_);

            return out_;
        }

        sf::Scalar getOutput() const { return out_; }
        //! The accumulated integral, so one loop can hand a settled trim to another.
        sf::Scalar getIntegrator() const { return iTerm_; }

    private:
        sf::Scalar kp_, ki_, kd_;
        sf::Scalar iBand_, iLimit_, outLimit_;
        sf::Scalar iTerm_;
        sf::Scalar out_;
    };
}

#endif

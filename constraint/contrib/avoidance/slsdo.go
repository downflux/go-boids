package avoidance

import (
	"math"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"

	dhr "github.com/downflux/go-database/geometry/hyperrectangle"
)

func SLSDOFeature(f feature.RO) constraint.Steer {
	return func(a agent.RO) vector.V {
		buf := vector.M{0, 0}

		d, n := dhr.Normal(f.AABB(), a.Position())

		buf.Copy(n)

		// As with SLSDO, this is already a steering force.
		//
		// N.B.: We are taking the absolute value of d - r, as it is possible
		// (through experimentation, for an agent to slightly penetrate the
		// AABB. The normal vector as reported by dhr.Normal still points
		// outwards, so we need to manually account for this change.
		e := a.MaxAcceleration() / math.Max(1e-5, math.Abs((d-a.Radius())))
		buf.Scale(e)

		return buf.V()
	}
}

// SLSDO calculates a steering acceleration given an agent-agent interaction.
// This is based on the slsdo avoidance algorithm. See
// https://slsdo.github.io/steering-behaviors/ for more information.
//
// N.B.: In the original slsdo algorithm, there is consideration for the
// time-to-collision (lb). We assume this value is accounted for by the caller,
// which should iterate over all local neighbors of the input agent a.
func SLSDO(o agent.RO) constraint.Steer {
	return func(a agent.RO) vector.V {
		am, om := a.Mass(), o.Mass()
		ap, av := a.Position(), a.Velocity()
		// Set the obstacle position a bit into the future. The base slsdo
		// implementation just uses o.Position here.
		op := vector.Add(o.Position(), o.Velocity())

		l := line.New(ap, av)
		buf := vector.M{0, 0}

		// Check that we are traveling in the direction of the opposing agent.
		if t := l.T(op); t > 0 {
			// Check if we will collide with the opposing agent. Add a small
			// buffer to the detection, as this only detects perpendicular
			// hits.
			r := 1.5 * (a.Radius() + o.Radius())
			if d := l.Distance(op); d < r {
				// Handle the case of a head-on collision. In this case, we
				// rotate the distance vector instead.
				if epsilon.Absolute(1e-5).Within(d, 0) {
					buf.Copy(op)
					buf.Sub(ap)
					buf.Copy(vector.V{-buf.Y(), buf.X()})
					buf.Unit()
				} else {
					// Calculate the projected distance vector between the
					// agent at passthrough point and the obstacle.
					buf.Copy(op)
					buf.Sub(l.L(t))
					buf.Unit()
				}

				// N.B.: buf here is already the steering vector. The
				// desired velocity can be calculated as
				//
				//  v' = a.V() + steer
				//
				// Since the steering vector is easier to calculate than
				// the desired velocity directly, we will skip
				// needlessly invoking the steering constraint.
				e := (r - d) / r * a.MaxAcceleration()
				e *= om / am
				buf.Scale(-e)
			}
		}

		return buf.V()
	}
}

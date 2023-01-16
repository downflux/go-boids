package separation

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/contrib/seek"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/filters"
	"github.com/downflux/go-database/flags/move"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

func Separation(db database.RO, r float64) constraint.Steer {
	return func(a agent.RO) vector.V {
		if a.MoveMode()&move.FSeparation == move.FNone {
			return vector.V{0, 0}
		}

		x, y := a.Position().X(), a.Position().Y()
		aabb := *hyperrectangle.New(
			vnd.V{x - r, y - r}, vnd.V{x + r, y + r},
		)

		obstacles := db.QueryAgents(aabb, func(b agent.RO) bool {
			// QueryAgents is an AABB query function -- we will
			// manually filter out agents which lie within the AABB
			// but outside the circle.
			if d := vector.Magnitude(vector.Sub(a.Position(), b.Position())); d > r {
				return false
			}
			return a.ID() != b.ID() && filters.AgentIsTeammate(a, b) && !filters.AgentOnDifferentLayers(a, b)
		})

		// Calculate the weighted average of the neighboring positions.
		// This value is our center-of-mass for the the entire group
		// (i.e. the desired velocity).  Therefore, we need to wrap this
		// in the steering constraint.
		sum := 0.0
		weights := make([]float64, 0, len(obstacles))
		vs := make([]vector.V, 0, len(obstacles))
		for _, o := range obstacles {
			m := o.Mass()
			sum += m
			weights = append(weights, m)
			vs = append(vs, vector.Sub(a.Position(), o.Position()))
		}

		buf := vector.M{0, 0}
		for i, w := range weights {
			buf.Add(vector.Scale(w/sum, vs[i]))
		}
		buf.Scale(1 - (sum+a.Mass())/a.Mass())
		buf.Add(a.Position())
		return seek.SLSDO(buf.V())(a)
	}
}

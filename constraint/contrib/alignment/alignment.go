package alignment

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/utils"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/filters"
	"github.com/downflux/go-database/flags/move"
	"github.com/downflux/go-geometry/2d/hyperrectangle"
	"github.com/downflux/go-geometry/2d/vector"
)

func Align(db database.RO, r float64) constraint.Steer {
	return utils.Steer(
		func(a agent.RO) vector.V {
			if a.MoveMode()&move.FAlignment == move.FNone {
				return a.Velocity()
			}

			x, y := a.Position().X(), a.Position().Y()
			aabb := *hyperrectangle.New(
				vector.V{x - r, y - r}, vector.V{x + r, y + r},
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

			// Calculate the weighted average of the neighboring
			// velocities. This value is our idealized velocity for
			// the entire group (i.e. the desired velocity).
			// Therefore, we need to wrap this in the steering
			// constraint.
			sum := 0.0
			weights := make([]float64, 0, len(obstacles))
			vs := make([]vector.V, 0, len(obstacles))
			for _, o := range obstacles {
				m := o.Mass()
				sum += m
				weights = append(weights, m)
				vs = append(vs, o.Velocity())
			}

			buf := vector.M{0, 0}
			for i, w := range weights {
				buf.Add(vector.Scale(w/sum, vs[i]))
			}
			buf.Scale(1 - (sum+a.Mass())/a.Mass())
			return buf.V()
		},
	)
}

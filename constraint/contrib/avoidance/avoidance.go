package avoidance

import (
	"sort"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/utils"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/filters"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

func Avoid(db *database.DB, r float64) constraint.Accelerator {
	return func(a agent.RO) vector.V {
		x, y := a.Position().X(), a.Position().Y()
		// Check for collision in the upcoming window.
		aabb := *hyperrectangle.New(
			vnd.V{x - r, y - r}, vnd.V{x + r, y + r},
		)

		// Use a clamping function to ensure some amount of action will
		// be taken in the case the agent is stuck between multiple
		// obstacles. See Reynolds 1987 for more information.
		//
		// TODO(minkezhang): Add feature avoidance.
		es := []e{}
		for _, obstacle := range db.QueryAgents(aabb, func(b agent.RO) bool {
			// QueryAgents is an AABB query function -- we will
			// manually filter out agents which lie within the AABB
			// but outside the circle.
			if d := vector.Magnitude(vector.Sub(a.Position(), b.Position())); d > r {
				return false
			}
			return a.ID() != b.ID() && !filters.AgentIsSquishable(a, b)
		}) {
			es = append(es, e{
				c: func(a agent.RO) vector.V { return SLSDO(a, obstacle) },
				h: vector.SquaredMagnitude(vector.Sub(a.Position(), obstacle.Position())),
			})
		}
		sort.Slice(es, func(i, j int) bool { return es[i].h < es[j].h })

		cs := make([]constraint.Accelerator, 0, len(es))
		for _, e := range es {
			cs = append(cs, e.c)
		}
		return utils.Clamped(cs, a.MaxAcceleration())(a)
	}
}

type e struct {
	c constraint.Accelerator
	h float64
}

package avoidance

import (
	"time"

	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/filters"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

func Avoid(db *database.DB, d time.Duration) constraint.Accelerator {
	t := float64(d / time.Second)

	return func(a agent.RO) vector.V {
		window := t * vector.Magnitude(a.Velocity())
		r := a.Radius() + window
		x, y := a.Position().X(), a.Position().Y()
		// Check for collision in the upcoming window.
		aabb := *hyperrectangle.New(
			vnd.V{
				x - r,
				y - r,
			}, vnd.V{
				x + r,
				y + r,
			},
		)

		// TODO(minkezhang): Only care about closest feature and / or
		// agent.
		//
		// TODO(minkezhang): Add clamped magnitude.
		//
		// TODO(minkezhang): Add feature avoidance.
		v := vector.M{0, 0}
		for _, obstacle := range db.QueryAgents(aabb, func(b agent.RO) bool {
			return a.ID() != b.ID() && !filters.AgentIsSquishable(a, b)
		}) {
			v.Add(SLSDO(a, obstacle))
		}

		return v.V()
	}
}

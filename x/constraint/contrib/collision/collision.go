package collision

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

func Collision(db *database.DB, r float64) constraint.Accelerator {
	return func(a agent.RO) vector.V {
		x, y := a.Position().X(), a.Position().Y()
		aabb := *hyperrectangle.New(
			vnd.V{
				x - r,
				x + r,
			}, vnd.V{
				y - r,
				y + r,
			},
		)

		v := vector.M{0, 0}
		for _, obstacle := range db.QueryFeatures(aabb, func(f feature.RO) bool { return true }) {
			v.Add(Feature(a, obstacle))
		}
		for _, obstacle := range db.QueryAgents(aabb, func(a agent.RO) bool { return true }) {
			v.Add(Agent(a, obstacle))
		}

		return v.V()
	}
}

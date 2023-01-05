package separation

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

func Separation(db *database.DB, r float64) constraint.Accelerator {
	return func(a agent.RO) vector.V {
		x, y := a.Position().X(), a.Position().Y()
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
		v := vector.M{0, 0}
		for _, obstacle := range db.QueryFeatures(aabb, func(f feature.RO) bool { return true }) {
			v.Add(Feature(a, obstacle))
		}
		for _, obstacle := range db.QueryAgents(aabb, func(b agent.RO) bool {
			return AgentFilter(a, b)
		}) {
			v.Add(SimpleAgentWithSingularity(a, obstacle))
		}

		return v.V()
	}
}

// TODO(minkezhang): Export Squishable, etc. in go-database.
func AgentFilter(a, b agent.RO) bool {
	return a.ID() != b.ID()
}

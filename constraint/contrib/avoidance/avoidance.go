package avoidance

import (
	"sort"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/utils"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-database/filters"
	"github.com/downflux/go-database/flags/move"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	dhr "github.com/downflux/go-database/geometry/hyperrectangle"
	vnd "github.com/downflux/go-geometry/nd/vector"
)

func Avoid(db database.RO, r float64) constraint.Steer {
	return func(a agent.RO) vector.V {
		if a.MoveMode()&move.FAvoidance == move.FNone {
			return vector.V{0, 0}
		}

		x, y := a.Position().X(), a.Position().Y()
		// Check for collision in the upcoming window.
		aabb := *hyperrectangle.New(
			vnd.V{x - r, y - r}, vnd.V{x + r, y + r},
		)

		// Use a clamping function to ensure some amount of action will
		// be taken in the case the agent is stuck between multiple
		// obstacles. See Reynolds 1987 for more information.
		es := []e{}
		for _, obstacle := range db.QueryFeatures(aabb, func(f feature.RO) bool {
			if !dhr.IntersectCircle(f.AABB(), a.Position(), r) {
				return false
			}
			return !filters.FeatureOnDifferentLayers(a, f)
		}) {
			d, _ := dhr.Normal(obstacle.AABB(), a.Position())
			es = append(es, e{
				c: SLSDOFeature(obstacle),
				h: d * d,
			})
		}
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
				c: SLSDO(obstacle),
				h: vector.SquaredMagnitude(vector.Sub(a.Position(), obstacle.Position())),
			})
		}
		sort.Slice(es, func(i, j int) bool { return es[i].h < es[j].h })

		cs := make([]constraint.Steer, 0, len(es))
		for _, e := range es {
			cs = append(cs, e.c)
		}
		return utils.Clamped(cs, a.MaxAcceleration())(a)
	}
}

type e struct {
	c constraint.Steer
	h float64
}

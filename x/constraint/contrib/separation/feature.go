package separation

import (
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-geometry/2d/vector"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

// Feature returns an acceleration vector away from the obstacle. This is a
// simplified collision calculation and models the rectangular feature as a
// circle. We may want to change this in the future to be more fine-grained.
func Feature(source agent.RO, obstacle feature.RO) vector.V {
	aabb := obstacle.AABB()
	mid := vector.V{
		(aabb.Max().X(vnd.AXIS_X) - aabb.Min().X(vnd.AXIS_X)) / 2,
		(aabb.Max().X(vnd.AXIS_Y) - aabb.Min().X(vnd.AXIS_Y)) / 2,
	}

	p := vector.Sub(source.Position(), mid)
	m := 1e-3
	return vector.Scale(m/vector.Magnitude(p), vector.Unit(p))
}

package collision

import (
	"math"

	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Agent(source agent.RO, obstacle agent.RO) vector.V {
	p := vector.Sub(obstacle.Position(), source.Position())
	r := source.Radius() + obstacle.Radius()
	separation := vector.Magnitude(p) / r
	m := obstacle.Mass() / source.Mass()
	return vector.Scale(m/math.Max(1e-5, separation-2), vector.Unit(p))
}

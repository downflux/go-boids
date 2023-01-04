package collision

import (
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Agent(source agent.RO, obstacle agent.RO) vector.V {
	p := vector.Sub(source.Position(), obstacle.Position())
	r := source.Radius() + obstacle.Radius()
	separation := vector.Magnitude(p) / r
	m := source.Mass() / obstacle.Mass()
	return vector.Scale(m/(separation-2), vector.Unit(p))
}

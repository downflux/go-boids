package separation

import (
	"math"

	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func SimpleAgentWithSingularity(source agent.RO, obstacle agent.RO) vector.V {
	f := vector.M{0, 0}
	f.Copy(source.Position())
	f.Sub(obstacle.Position())

	r := source.Radius() + obstacle.Radius()
	m := obstacle.Mass() / source.Mass()
	separation := vector.Magnitude(f.V())/r - 1
	f.Unit()
	f.Scale(m / math.Max(1e-5, separation))

	return f.V()
}

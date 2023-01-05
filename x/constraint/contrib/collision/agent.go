package collision

import (
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func SimpleAgent(source agent.RO, obstacle agent.RO) vector.V {
	if vector.Dot(source.Velocity(), obstacle.Velocity()) >= 0 {
		return vector.V{0, 0}
	}

	f := vector.M{0, 0}
	f.Copy(source.Position())
	f.Sub(obstacle.Position())
	f.Scale(1 / vector.SquaredMagnitude(f.V()))

	m := obstacle.Mass() / source.Mass()
	f.Scale(m)

	return f.V()
}

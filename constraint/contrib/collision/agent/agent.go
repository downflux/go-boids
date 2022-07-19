package agent

import (
	"math"
	"math/rand"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

var _ constraint.C = C{}

type C struct {
	o O
}

type O struct {
	Obstacle agent.RO
}

func New(o O) *C {
	return &C{
		o: o,
	}
}
func jitter() vector.V {
	return vector.Unit(
		*vector.New(
			rand.Float64()+1e-5,
			rand.Float64()+1e-5,
		),
	)
}
func (c C) Accelerate(a agent.RO) vector.V {
	v := vector.Sub(a.V(), c.o.Obstacle.V())
	d := *hypersphere.New(c.o.Obstacle.P(), c.o.Obstacle.R()+a.R())

	l := line.New(a.P(), v)
	if pmin, pmax, ok := l.IntersectCircle(d); ok {
		t := math.Inf(0)
		var p vector.V
		for _, pc := range []vector.V{pmin, a.P(), pmax} {
			if tc := l.T(pc); tc >= -1 && tc <= 1 && tc < t {
				t = tc
				p = pc
			}
		}

		if p != nil {
			q := vector.Sub(l.L(t), c.o.Obstacle.P())
			if epsilon.Within(vector.Magnitude(p), 0) {
				q = jitter()
			}

			separation := vector.Magnitude(q) / d.R()
			return vector.Scale(
				math.Max(1e-5, 1/(separation-d.R())),
				vector.Unit(vector.Sub(p, c.o.Obstacle.P())),
			)
		}
	}
	return *vector.New(0, 0)
}

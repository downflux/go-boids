package steering

import (
	"math"
	"testing"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"

	mock "github.com/downflux/go-boids/agent/mock"
)

func TestS(t *testing.T) {
	configs := []struct {
		name  string
		a     agent.RO
		force vector.V
		tau   float64
		want  vector.V
	}{
		{
			name: "V=X+/Heading=X+/Within",
			a: mock.New(mock.O{
				MaxAcceleration: *polar.New(math.Inf(0), math.Inf(0)),
				V:               *vector.New(100, 0),
				Heading:         *polar.New(1, 0),
				MaxVelocity:     *polar.New(100, math.Inf(0)),
			}),
			force: *vector.New(0, 100),
			want:  vector.Scale(math.Sqrt(2)/2, *vector.New(-100, 100)),
			tau:   1,
		},
		{
			name: "V=0/Heading=X+/Within",
			a: mock.New(mock.O{
				MaxAcceleration: *polar.New(math.Inf(0), math.Inf(0)),
				V:               *vector.New(0, 0),
				Heading:         *polar.New(1, 0),
				MaxVelocity:     *polar.New(100, math.Inf(0)),
			}),
			force: *vector.New(100, 100),
			want:  vector.Scale(math.Sqrt(2)/2, *vector.New(100, 100)),
			tau:   1,
		},
		{
			name: "V=0/Heading=X+/Within/Mirror",
			a: mock.New(mock.O{
				MaxAcceleration: *polar.New(math.Inf(0), math.Inf(0)),
				V:               *vector.New(0, 0),
				Heading:         *polar.New(1, 0),
				MaxVelocity:     *polar.New(100, math.Inf(0)),
			}),
			force: *vector.New(100, -100),
			want:  vector.Scale(math.Sqrt(2)/2, *vector.New(100, -100)),
			tau:   1,
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := S(c.a, c.force, c.tau); !vector.Within(got, c.want) {
				t.Errorf("S() = %v, want = %v", got, c.want)
			}
		})
	}
}

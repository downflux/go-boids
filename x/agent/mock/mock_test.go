package mock

import (
	"math"
	"testing"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func TestStep(t *testing.T) {
	configs := []struct {
		name         string
		a            agent.RW
		acceleration vector.V
		tau          float64
		want         agent.RO
	}{
		{
			name: "Stationary",
			a: New(O{
				V:       *vector.New(0, 0),
				P:       *vector.New(100, 100),
				Heading: *polar.New(1, math.Pi/2),
			}),
			acceleration: *vector.New(0, 0),
			tau:          0.5,
			want: New(O{
				V:       *vector.New(0, 0),
				P:       *vector.New(100, 100),
				Heading: *polar.New(1, math.Pi/2),
			}),
		},
		{
			name: "NoAcceleration",
			a: New(O{
				V:       *vector.New(10, -10),
				P:       *vector.New(100, 100),
				Heading: *polar.New(1, math.Pi/2),
			}),
			acceleration: *vector.New(0, 0),
			tau:          0.5,
			want: New(O{
				V:       *vector.New(10, -10),
				P:       *vector.New(105, 95),
				Heading: *polar.New(1, -math.Pi/4),
			}),
		},
		{
			name: "Acceleration",
			a: New(O{
				V:       *vector.New(10, -10),
				P:       *vector.New(100, 100),
				Heading: *polar.New(1, math.Pi/2),
			}),
			acceleration: *vector.New(20, -20),
			tau:          0.5,
			want: New(O{
				V:       *vector.New(20, -20),
				P:       *vector.New(110, 90),
				Heading: *polar.New(1, -math.Pi/4),
			}),
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			agent.Step(c.a, c.acceleration, c.tau)
			t.Run("P", func(t *testing.T) {
				if got := c.a.P(); !vector.Within(got, c.want.P()) {
					t.Errorf("P() = %v, want = %v", got, c.want.P())
				}
			})
			t.Run("V", func(t *testing.T) {
				if got := c.a.V(); !vector.Within(got, c.want.V()) {
					t.Errorf("V() = %v, want = %v", got, c.want.V())
				}
			})
			t.Run("Heading", func(t *testing.T) {
				if got := c.a.Heading(); !polar.Within(got, c.want.Heading()) {
					t.Errorf("P() = %v, want = %v", got, c.want.Heading())
				}
			})
		})
	}
}

package locomotion

import (
	"math"
	"testing"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"

	mock "github.com/downflux/go-boids/agent/mock"
)

func TestL(t *testing.T) {
	configs := []struct {
		name     string
		a        agent.RO
		steering vector.V
		tau      float64
		want     agent.RO
	}{
		{
			name: "Stationary",
			a: mock.New(mock.O{
				P:           *vector.New(1, 0),
				V:           *vector.New(1, 0),
				Heading:     *polar.New(1, 0),
				MaxVelocity: *polar.New(100, 1),
			}),
			steering: *vector.New(-1, 0),
			tau:      1,
			want: mock.New(mock.O{
				P:       *vector.New(1, 0),
				V:       *vector.New(0, 0),
				Heading: *polar.New(1, 0),
			}),
		},
		{
			name: "Turn",
			a: mock.New(mock.O{
				P:           *vector.New(1, 0),
				V:           *vector.New(1, 0),
				Heading:     *polar.New(1, 0),
				MaxVelocity: *polar.New(100, math.Pi),
			}),
			steering: *vector.New(-1, 1),
			tau:      1,
			want: mock.New(mock.O{
				P:       *vector.New(1, 1),
				V:       *vector.New(0, 1),
				Heading: *polar.New(1, math.Pi/2),
			}),
		},
		{
			name: "Turn/Clockwise",
			a: mock.New(mock.O{
				P:           *vector.New(1, 0),
				V:           *vector.New(0, 1),
				Heading:     *polar.New(1, math.Pi/2),
				MaxVelocity: *polar.New(100, math.Pi),
			}),
			steering: *vector.New(1, -1),
			tau:      1,
			want: mock.New(mock.O{
				P:       *vector.New(2, 0),
				V:       *vector.New(1, 0),
				Heading: *polar.New(1, 0),
			}),
		},
		{
			name: "Turn/Trunate",
			a: mock.New(mock.O{
				P:           *vector.New(1, 0),
				V:           *vector.New(1, 0),
				Heading:     *polar.New(1, 0),
				MaxVelocity: *polar.New(100, math.Pi/2),
			}),
			steering: vector.Sub(
				vector.Rotate(2.0/3.0*math.Pi, *vector.New(1, 0)),
				*vector.New(1, 0),
			),
			tau: 1,
			want: mock.New(mock.O{
				P:       *vector.New(1, 1),
				V:       *vector.New(0, 1),
				Heading: *polar.New(1, math.Pi/2),
			}),
		},
		{
			name: "Turn/Trunate/Clockwise",
			a: mock.New(mock.O{
				P:           *vector.New(1, 0),
				V:           *vector.New(0, 1),
				Heading:     *polar.New(1, math.Pi/2),
				MaxVelocity: *polar.New(100, math.Pi/2),
			}),
			steering: vector.Sub(
				vector.Rotate(-math.Pi/6.0, *vector.New(1, 0)),
				*vector.New(0, 1),
			),
			tau: 1,
			want: mock.New(mock.O{
				P:       *vector.New(2, 0),
				V:       *vector.New(1, 0),
				Heading: *polar.New(1, 0),
			}),
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			got := L(c.a, c.steering, c.tau)
			t.Run("P", func(t *testing.T) {
				if !vector.WithinEpsilon(got.P(), c.want.P(), epsilon.Absolute(1e-5)) {
					t.Errorf("P() = %v, want = %v", got.P(), c.want.P())
				}
			})
			t.Run("V", func(t *testing.T) {
				if !vector.WithinEpsilon(got.V(), c.want.V(), epsilon.Absolute(1e-5)) {
					t.Errorf("V() = %v, want = %v", got.V(), c.want.V())
				}
			})
			t.Run("Heading", func(t *testing.T) {
				if !polar.WithinEpsilon(got.Heading(), c.want.Heading(), epsilon.Absolute(1e-5)) {
					t.Errorf("Heading() = %v, want = %v", got.Heading(), c.want.Heading())
				}
			})
		})
	}
}

package agent

import (
	"testing"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"

	mock "github.com/downflux/go-boids/agent/mock"
)

func TestForce(t *testing.T) {
	configs := []struct {
		name     string
		obstacle agent.A
		agent    agent.A
		k        float64
		tau      float64
		maxRange float64
		want     vector.V
	}{
		{
			name: "Miss/RelativeStationary",
			obstacle: mock.New(mock.O{
				P:    *vector.New(0, 0),
				V:    *vector.New(1, 0),
				R:    1,
				Mass: 1,
			}),
			agent: mock.New(mock.O{
				P:    *vector.New(3, 0),
				V:    *vector.New(1, 0),
				R:    1,
				Mass: 1,
			}),
			k:        1,
			tau:      1,
			maxRange: 2,
			want:     *vector.New(0, 0),
		},
		{
			name: "Miss/Slide",
			obstacle: mock.New(mock.O{
				P:    *vector.New(0, 0),
				V:    *vector.New(1, 0),
				R:    1,
				Mass: 1,
			}),
			agent: mock.New(mock.O{
				P:    *vector.New(0, 4),
				V:    *vector.New(-1, 0),
				R:    1,
				Mass: 1,
			}),
			k:        1,
			tau:      1,
			maxRange: 2,
			want:     *vector.New(0, 0),
		},
		{
			name: "Collide/Direct",
			obstacle: mock.New(mock.O{
				P:    *vector.New(0, 0),
				V:    *vector.New(1, 0),
				R:    1,
				Mass: 1,
			}),
			agent: mock.New(mock.O{
				P:    *vector.New(3, 0),
				V:    *vector.New(-1, 0),
				R:    1,
				Mass: 1,
			}),
			k:        1,
			tau:      1,
			maxRange: 2,
			want:     *vector.New(1e10, 0),
		},
		{
			name: "Collide/Direct/NonTouching",
			obstacle: mock.New(mock.O{
				P:    *vector.New(-0.1, 0),
				V:    *vector.New(1, 0),
				R:    1,
				Mass: 1,
			}),
			agent: mock.New(mock.O{
				P:    *vector.New(3, 0),
				V:    *vector.New(-1, 0),
				R:    1,
				Mass: 1,
			}),
			k:        1,
			tau:      1,
			maxRange: 10,
			want:     *vector.New(1e10, 0),
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := New(O{
				Obstacle: c.obstacle,
				K:        c.k,
				Tau:      c.tau,
				MaxRange: c.maxRange,
			}).Force(c.agent); !vector.Within(got, c.want) {
				t.Errorf("A() = %v, want = %v", got, c.want)
			}
		})
	}
}

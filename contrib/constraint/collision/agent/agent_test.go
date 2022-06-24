package agent

import (
	"testing"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"

	mock "github.com/downflux/go-boids/agent/mock"
)

func TestA(t *testing.T) {
	configs := []struct {
		name     string
		obstacle agent.A
		agent    agent.A
		k        float64
		want     vector.V
	}{
		{
			name: "Miss/Stationary",
			obstacle: mock.New(mock.O{
				P: *vector.New(0, 0),
				V: *vector.New(0, 0),
				R: 1,
			}),
			agent: mock.New(mock.O{
				P: *vector.New(3, 0),
				V: *vector.New(0, 0),
				R: 1,
			}),
			k:    10,
			want: *vector.New(0, 0),
		},
		{
			name: "Miss/Slide",
			obstacle: mock.New(mock.O{
				P: *vector.New(0, 0),
				V: *vector.New(1, 0),
				R: 1,
			}),
			agent: mock.New(mock.O{
				P: *vector.New(0, 3),
				V: *vector.New(-1, 0),
				R: 1,
			}),
			k:    10,
			want: *vector.New(0, 0),
		},
		{
			name: "Collide/Direct",
			obstacle: mock.New(mock.O{
				P: *vector.New(0, 0),
				V: *vector.New(1, 0),
				R: 1,
			}),
			agent: mock.New(mock.O{
				P: *vector.New(3, 0),
				V: *vector.New(-1, 0),
				R: 1,
			}),
			k:    10,
			want: *vector.New(10, 0),
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := New(O{
				Obstacle: c.obstacle,
				K:        c.k,
			}).A(c.agent); !vector.Within(got, c.want) {
				t.Errorf("A() = %v, want = %v", got, c.want)
			}
		})
	}
}

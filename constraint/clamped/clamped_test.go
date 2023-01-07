package clamped

import (
	"testing"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"

	magent "github.com/downflux/go-database/agent/mock"
)

func mock(v vector.V) constraint.Accelerator {
	return func(agent.RO) vector.V { return v }
}

func TestClamped(t *testing.T) {
	type config struct {
		name         string
		accelerators []constraint.Accelerator
		limit        float64
		want         vector.V
	}

	configs := []config{
		{
			name:         "Empty",
			accelerators: nil,
			limit:        0,
			want:         vector.V{0, 0},
		},
		{
			name: "Trivial",
			accelerators: []constraint.Accelerator{
				mock(vector.V{1, 1}),
			},
			limit: 10,
			want:  vector.V{1, 1},
		},
		{
			name: "TooLarge",
			accelerators: []constraint.Accelerator{
				mock(vector.V{100, 0}),
			},
			limit: 10,
			want:  vector.V{10, 0},
		},
		{
			name: "Truncate",
			accelerators: []constraint.Accelerator{
				mock(vector.V{-1, 0}),
				mock(vector.V{10, 0}),
			},
			limit: 10,
			want:  vector.V{8, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			got := Clamped(c.accelerators, c.limit)(&magent.A{})
			if !vector.Within(got, c.want) {
				t.Errorf("Clamped() = %v, want = %v", got, c.want)
			}
		})
	}
}

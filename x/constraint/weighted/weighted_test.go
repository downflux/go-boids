package weighted

import (
	"testing"

	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"

	magent "github.com/downflux/go-database/agent/mock"
)

func mock(v vector.V) constraint.Accelerator {
	return func(agent.RO) vector.V { return v }
}

func TestWeightedAverage(t *testing.T) {
	type config struct {
		name         string
		accelerators []constraint.Accelerator
		weights      []float64
		want         vector.V
	}

	configs := []config{
		{
			name:         "Trivial",
			accelerators: []constraint.Accelerator{},
			weights:      []float64{},
			want:         vector.V{0, 0},
		},
		{
			name: "Simple",
			accelerators: []constraint.Accelerator{
				mock(vector.V{0, 2}),
				mock(vector.V{2, 0}),
			},
			weights: []float64{1, 1},
			want:    vector.V{1, 1},
		},
		{
			name: "FilterNoContrib",
			accelerators: []constraint.Accelerator{
				mock(vector.V{0, 0}),
				mock(vector.V{2, 0}),
			},
			weights: []float64{1, 1},
			want:    vector.V{2, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			got := WeightedAverage(c.accelerators, c.weights)(&magent.A{})
			if !vector.Within(got, c.want) {
				t.Errorf("WeightedAverage() = %v, want = %v", got, c.want)
			}
		})
	}
}

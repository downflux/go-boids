package utils

import (
	"testing"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/utils/mock"
	"github.com/downflux/go-geometry/2d/vector"

	magent "github.com/downflux/go-database/agent/mock"
)

func TestWeightedAverage(t *testing.T) {
	type config struct {
		name      string
		steerings []constraint.Steer
		weights   []float64
		want      vector.V
	}

	configs := []config{
		{
			name:      "Trivial",
			steerings: []constraint.Steer{},
			weights:   []float64{},
			want:      vector.V{0, 0},
		},
		{
			name: "Simple",
			steerings: []constraint.Steer{
				mock.M(vector.V{0, 2}),
				mock.M(vector.V{2, 0}),
			},
			weights: []float64{1, 1},
			want:    vector.V{1, 1},
		},
		{
			name: "FilterNoContrib",
			steerings: []constraint.Steer{
				mock.M(vector.V{0, 0}),
				mock.M(vector.V{2, 0}),
			},
			weights: []float64{1, 1},
			want:    vector.V{2, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			got := WeightedAverage(c.steerings, c.weights)(&magent.A{})
			if !vector.Within(got, c.want) {
				t.Errorf("WeightedAverage() = %v, want = %v", got, c.want)
			}
		})
	}
}

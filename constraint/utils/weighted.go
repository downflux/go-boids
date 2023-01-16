package utils

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

func Average(steerings []constraint.Steer) constraint.Steer {
	ws := make([]float64, 0, len(steerings))
	for i := 0; i < len(ws); i++ {
		ws[i] = 1
	}
	return WeightedAverage(steerings, ws)
}

func WeightedAverage(steerings []constraint.Steer, weights []float64) constraint.Steer {
	if len(steerings) != len(weights) {
		panic("mismatching accelerator and weight lengths")
	}

	return func(a agent.RO) vector.V {
		sum := 0.0
		v := vector.M{0, 0}

		for i, s := range steerings {
			u := vector.M{0, 0}
			u.Copy(s(a))

			// We will only track steerings which contribute to
			// the total.
			if !epsilon.Absolute(1e-5).Within(vector.SquaredMagnitude(u.V()), 0) {
				sum += weights[i]
				u.Scale(weights[i])
			}
			v.Add(u.V())
		}

		if !epsilon.Within(sum, 0) {
			v.Scale(1 / sum)
		}

		return v.V()
	}
}

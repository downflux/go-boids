package weighted

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

func Average(accelerators []constraint.Accelerator) constraint.Accelerator {
	ws := make([]float64, 0, len(accelerators))
	for i := 0; i < len(ws); i++ {
		ws[i] = 1
	}
	return WeightedAverage(accelerators, ws)
}

func WeightedAverage(accelerators []constraint.Accelerator, weights []float64) constraint.Accelerator {
	if len(accelerators) != len(weights) {
		panic("mismatching accelerator and weight lengths")
	}

	return func(a agent.RO) vector.V {
		sum := 0.0
		v := vector.M{0, 0}

		for i, accel := range accelerators {
			u := vector.M{0, 0}
			u.Copy(accel(a))

			// We will only track accelerators which contribute to
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

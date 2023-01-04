package weighted

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

const (
	tolerance = 1e-5
)

func GenerateAverage(accelerators []constraint.Accelerator) constraint.Accelerator {
	ws := make([]float64, 0, len(accelerators))
	for i := 0; i < len(ws); i++ {
		ws[i] = 1
	}
	return GenerateWeightedAverage(accelerators, ws)
}

func GenerateWeightedAverage(accelerators []constraint.Accelerator, weights []float64) constraint.Accelerator {
	if len(accelerators) != len(weights) {
		panic("mismatching accelerator and weight lengths")
	}

	sum := 0.0
	for _, w := range weights {
		sum += w
	}

	return func(a agent.RO) vector.V {
		v := vector.M{0, 0}
		for i, accel := range accelerators {
			u := vector.M{0, 0}
			u.Copy(accel(a))
			// We are using the weights to scale each vector instead
			// of its innate magnitude.
			if !epsilon.Absolute(tolerance).Within(vector.SquaredMagnitude(u.V()), 0) {
				u.Unit()
				u.Scale(weights[i] / sum)
			}
			v.Add(u.V())
		}

		return v.V()
	}
}

package config

import (
	"fmt"
	"math"
	"math/rand"

	"github.com/downflux/go-boids/x/agent/mock"
	"github.com/downflux/go-geometry/2d/vector"
)

type C struct {
	Agents    []*mock.A
	Height    float64
	Width     float64
	MaxRadius float64
}

func rn(min, max float64) float64 { return min + (max-min)*rand.Float64() }

func rv(min, max float64) vector.V {
	return vector.Scale(
		rn(min, max),
		vector.Unit(*vector.New(rn(-1, 1), rn(-1, 1))),
	)
}

func GenerateGrid(h int, w int) C {
	const tile = 50.0
	c := &C{
		Height: tile * (float64(h) + 1.0),
		Width:  tile * (float64(w) + 1.0),
	}

	var positions []vector.V
	var goals []vector.V

	for i := 0; i < h; i++ {
		for j := 0; j < w; j++ {
			positions = append(positions, *vector.New(float64(i+1)*tile, float64(j+1)*tile))
			goals = append(goals, *vector.New(float64(i+1)*tile, float64(j+1)*tile))
		}
	}
	rand.Shuffle(len(goals), func(i, j int) { goals[i], goals[j] = goals[j], goals[i] })

	for _, p := range positions {
		c.Agents = append(c.Agents, mock.Lamborghini(mock.O{
			ID: mock.DebugID(fmt.Sprintf("(%.2f, %.2f)", p.X(), p.Y())),
			P:  p,
			V:  rv(-20, 20),
		}))
	}

	r := math.Inf(-1)
	for range c.Agents {
		r = math.Max(r, 0)
	}
	c.MaxRadius = r

	return *c
}

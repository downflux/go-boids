package config

import (
	"fmt"
	"math"
	"math/rand"

	"github.com/downflux/go-boids/agent/mock"
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

func GenerateCollision() C {
	return C{
		Height:    200,
		Width:     200,
		MaxRadius: 10,
		Agents: []*mock.A{
			mock.Lamborghini(mock.O{
				ID:   mock.DebugID("A"),
				P:    *vector.New(50, 100),
				V:    *vector.New(10, 0),
				Goal: *vector.New(100, 100),
			}),
			mock.Lamborghini(mock.O{
				ID:   mock.DebugID("B"),
				P:    *vector.New(70, 100),
				V:    *vector.New(-10, 0),
				Goal: *vector.New(0, 100),
			}),
			mock.Lamborghini(mock.O{
				ID:   mock.DebugID("C"),
				P:    *vector.New(65, 130),
				V:    *vector.New(0, -10),
				Goal: *vector.New(65, 20),
			}),
		},
	}
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

	for i, p := range positions {
		c.Agents = append(c.Agents, mock.Lamborghini(mock.O{
			ID:   mock.DebugID(fmt.Sprintf("%v", i)),
			P:    p,
			V:    rv(-20, 20),
			Goal: goals[i],
		}))
	}

	r := math.Inf(-1)
	for _, a := range c.Agents {
		r = math.Max(r, a.R())
	}
	c.MaxRadius = r

	return *c
}

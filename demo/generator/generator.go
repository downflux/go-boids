package main

import (
	"encoding/json"
	"flag"
	"math/rand"
	"os"

	"github.com/downflux/go-boids/demo/config"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var (
	// MaxAcceleration is the maximum impuse that can be generated over some
	// time period tau. Note that this should be fairly large compared to
	// MaxSpeed to ensure agents can stop in time to avoid collisions.
	MaxAcceleration = *polar.New(10, 0)
	MaxSpeed        = 1.0
	Radius          = 5

	fn = flag.String("out", "/dev/stdout", "")
)

func rn(min, max float64) float64 { return min + (max-min)*rand.Float64() }

func rv(min, max float64) vector.V {
	return vector.Scale(
		rn(min, max),
		vector.Unit(*vector.New(rn(-1, 1), rn(-1, 1))),
	)
}

func GenerateGrid(h int, w int) config.C {
	const tile = 50.0
	c := &config.C{}

	var positions []vector.V
	var goals []vector.V

	for i := 0; i < h; i++ {
		for j := 0; j < w; j++ {
			positions = append(positions, *vector.New(float64(i)*tile, float64(j)*tile))
			goals = append(goals, *vector.New(float64(i)*tile, float64(j)*tile))
		}
	}

	rand.Shuffle(len(goals), func(i, j int) { goals[i], goals[j] = goals[j], goals[i] })

	for i, p := range positions {
		c.Agents = append(c.Agents, &config.A{
			O: config.O{
				P:               p,
				V:               rv(-0.5, 0.5),
				R:               float64(Radius),
				Goal:            goals[i],
				MaxAcceleration: MaxAcceleration,
				MaxSpeed:        MaxSpeed,
			},
		})
	}
	return *c
}

func main() {
	flag.Parse()

	fp, err := os.OpenFile(*fn, os.O_RDWR|os.O_CREATE, 0666)
	if err != nil {
		panic(err)
	}
	defer fp.Close()

	c := GenerateGrid(10, 10)

	e := json.NewEncoder(fp)
	e.SetIndent("", "  ")

	_ = e.Encode(&c)
}

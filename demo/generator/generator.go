package main

import (
	"encoding/json"
	"flag"
	"math"
	"math/rand"
	"os"

	"github.com/downflux/go-boids/demo/config"
	"github.com/downflux/go-geometry/2d/vector"
)

var (
	// MaxNetForce is the maximum impuse that can be generated over some
	// time period tau. Note that this should be fairly large compared to
	// MaxSpeed to ensure agents can stop in time to avoid collisions.
	//
	// Due to the scale of our simulations, our net force and max speed
	// values are not at human-scale.
	MaxNetForce = 1000.0
	MaxSpeed    = 600.0
	Radius      = 5

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
		mass := rn(10, 15)
		radius := float64(Radius) * math.Pow(mass/10.0, 2)
		speed := MaxSpeed / mass
		velocity := rv(-0.5, 0.5)
		heading := map[bool]vector.V{
			true:  *vector.New(1, 0),
			false: vector.Unit(velocity),
		}[vector.Within(velocity, *vector.New(0, 0))]
		c.Agents = append(c.Agents, &config.A{
			O: config.O{
				P:           p,
				V:           velocity,
				R:           radius,
				Goal:        goals[i],
				Mass:        mass,
				Heading:     heading,
				MaxNetForce: MaxNetForce,
				MaxSpeed:    speed,
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

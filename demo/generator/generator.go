package main

import (
	"encoding/json"
	"fmt"
	"flag"
	"math"
	"math/rand"
	"os"

	"github.com/downflux/go-boids/demo/config"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var (
	// MaxNetForce is the maximum impuse that can be generated over some
	// time period tau. Note that this should be fairly large compared to
	// MaxSpeed to ensure agents can stop in time to avoid collisions.
	//
	// Due to the scale of our simulations, our net force and max speed
	// values are not at human-scale.
	MaxNetForce  = 10.0
	MaxNetTorque = 10.0
	MaxVelocity  = *polar.New(600.0, 10*math.Pi)
	Radius       = 5.0

	fn = flag.String("out", "/dev/stdout", "")
	mode = flag.String("mode", "grid", "")
)

func rn(min, max float64) float64 { return min + (max-min)*rand.Float64() }

func rv(min, max float64) vector.V {
	return vector.Scale(
		rn(min, max),
		vector.Unit(*vector.New(rn(-1, 1), rn(-1, 1))),
	)
}

func GenerateSimple() config.C {
	return config.C{
		Height: 200,
		Width: 200,
		Agents: []*config.A{
			&config.A{O: config.O{
				P:            *vector.New(25, 50),
				V:            *vector.New(0, 0),
				R:            Radius,
				Goal:         *vector.New(175, 50),
				Mass:         10,
				Heading:      *polar.New(1, 0),
				MaxNetTorque: MaxNetTorque,
				MaxNetForce:  MaxNetForce,
				MaxVelocity:  MaxVelocity,
			}},
			&config.A{O: config.O{
				P:            *vector.New(175, 50),
				V:            *vector.New(0, 0),
				R:            Radius,
				Goal:         *vector.New(25, 50),
				Mass:         10,
				Heading:      *polar.New(1, math.Pi),
				MaxNetTorque: MaxNetTorque,
				MaxNetForce:  MaxNetForce,
				MaxVelocity:  MaxVelocity,
			}},
		},
	}
}

func GenerateGrid(h int, w int) config.C {
	const tile = 50.0
	c := &config.C{
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
		mass := rn(10, 15)
		radius := float64(Radius) * math.Pow(mass/10.0, 2)
		maxVelocity := *polar.New(
			MaxVelocity.R()/mass,
			MaxVelocity.Theta()/mass,
		)
		velocity := rv(-0.5, 0.5)
		heading := map[bool]polar.V{
			true:  *polar.New(1, 0),
			false: polar.Polar(vector.Unit(velocity)),
		}[vector.Within(velocity, *vector.New(0, 0))]
		c.Agents = append(c.Agents, &config.A{
			O: config.O{
				P:            p,
				V:            velocity,
				R:            radius,
				Goal:         goals[i],
				Mass:         mass,
				Heading:      heading,
				MaxNetTorque: MaxNetTorque,
				MaxNetForce:  MaxNetForce,
				MaxVelocity:  maxVelocity,
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

	c, ok := map[string]config.C{
		"grid": GenerateGrid(10, 10),
		"simple": GenerateSimple(),
	}[*mode]
	if !ok {
		panic(fmt.Sprintf("unsupported mode %v", *mode))
	}

	e := json.NewEncoder(fp)
	e.SetIndent("", "  ")

	_ = e.Encode(&c)
}

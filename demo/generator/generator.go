package main

import (
	"encoding/json"
	"flag"
	"math/rand"
	"os"

	"github.com/downflux/go-boids/demo/config"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/cylindrical"
	"github.com/downflux/go-geometry/2d/vector"
)

var (
	MaxAcceleration = *cylindrical.New(1, 0)
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
	for i := 0; i < h; i++ {
		for j := 0; j < w; j++ {
			c.Agents = append(c.Agents, &config.A{
				O: config.O{
					P:               *vector.New(float64(i)*tile, float64(j)*tile),
					V:               rv(-0.5, 0.5),
					R:               float64(Radius),
					MaxAcceleration: MaxAcceleration,
					MaxSpeed:        MaxSpeed,
				},
			})
		}
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

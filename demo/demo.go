package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"unsafe"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/boid"
	"github.com/downflux/go-boids/demo/config"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"

	bkd "github.com/downflux/go-boids/kd"
	v2d "github.com/downflux/go-geometry/2d/vector"
)

var (
	fnInput  = flag.String("in", "/dev/stdin", "")
	fnOutput = flag.String("out", "/dev/stdout", "")
)

type P config.A

func (p *P) P() vector.V    { return vector.V(p.Agent().P()) }
func (p *P) Agent() agent.A { return (*config.A)(unsafe.Pointer(p)) }

type Environment config.C

func (e Environment) Points() []point.P {
	ps := make([]point.P, 0, len(config.C(e).Agents))
	for _, a := range config.C(e).Agents {
		p := P(a)
		ps = append(ps, &p)
	}
	return ps
}

func generate(fn string) Environment {
	fp, err := os.Open(fn)
	if err != nil {
		panic(fmt.Sprintf("could not open config file: %v", err))
	}
	defer fp.Close()

	c := config.C{}
	if err := json.NewDecoder(fp).Decode(&c); err != nil {
		panic(fmt.Sprintf("could not decode config file: %v", err))
	}
	return Environment(c)
}

func main() {
	e := generate(*fnInput)
	const frame = 1.0 / 60.0

	t, err := kd.New(e.Points())
	if err != nil {
		panic(fmt.Sprintf("could not construct tree: %v", err))
	}
	for f := 0; f < 1e3; f++ {
		mutations := boid.Step(boid.O{
			T:   bkd.Lift(t),
			Tau: .9,
		})
		for _, m := range mutations {
			a := m.Agent.(*config.A)
			a.SetV(v2d.Add(a.V(), v2d.Scale(frame, m.Acceleration)))
			a.SetP(v2d.Add(a.V(), v2d.Scale(frame, a.V())))
		}
	}
}

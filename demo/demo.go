package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/gif"
	"math"
	"os"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/boid"
	"github.com/downflux/go-boids/demo/config"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"

	bkd "github.com/downflux/go-boids/kd"
	v2d "github.com/downflux/go-geometry/2d/vector"
	examplesdraw "github.com/downflux/go-orca/examples/draw"
)

var (
	// Color palette for drawing.

	black = color.Black
	white = color.White
	red   = color.RGBA{255, 0, 0, 255}
	green = color.RGBA{0, 255, 0, 255}
	blue  = color.RGBA{0, 0, 255, 255}
	gray  = color.RGBA{192, 192, 192, 255}

	fnInput  = flag.String("in", "/dev/stdin", "")
	fnOutput = flag.String("out", "/dev/stdout", "")
	n        = flag.Int("frames", 1500, "")

	margin = *v2d.New(50, 50)
)

var (
	_ point.P = &P{}
	_ bkd.P   = &P{}
)

type P config.A

func (p *P) P() vector.V    { return vector.V(p.Agent().P()) }
func (p *P) Agent() agent.A { return (*config.A)(p) }

type Environment config.C

func (e Environment) Points() []point.P {
	ps := make([]point.P, 0, len(config.C(e).Agents))
	for _, a := range config.C(e).Agents {
		p := P(a)
		ps = append(ps, &p)
	}
	return ps
}

func (e Environment) Bound() hyperrectangle.R {
	min := *v2d.New(math.Inf(0), math.Inf(0))
	max := *v2d.New(math.Inf(-1), math.Inf(-1))

	for _, p := range e.Points() {
		min = *v2d.New(
			math.Min(min.X(), v2d.V(p.P()).X()),
			math.Min(min.Y(), v2d.V(p.P()).Y()),
		)
		max = *v2d.New(
			math.Max(max.X(), v2d.V(p.P()).X()),
			math.Max(max.Y(), v2d.V(p.P()).Y()),
		)
	}
	return *hyperrectangle.New(
		*vector.New(0, 0),
		vector.V(v2d.Add(
			v2d.Sub(max, min),
			v2d.Scale(2, margin),
		)),
	)
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
	flag.Parse()

	e := generate(*fnInput)
	const frame = 1.0 / 60.0

	t, err := kd.New(e.Points())
	if err != nil {
		panic(fmt.Sprintf("could not construct tree: %v", err))
	}

	// trailbuf keeps the last N positions of agents in memory for
	// visualization.
	var trailbuf [50][]v2d.V
	var frames []*image.Paletted
	b := e.Bound()

	for i := 0; i < *n; i++ {
		// Overwrite trail buffer.
		trailbuf[i%len(trailbuf)] = nil
		for _, p := range e.Points() {
			trailbuf[i%len(trailbuf)] = append(trailbuf[i%len(trailbuf)], p.(*P).Agent().P())
		}

		mutations := boid.Step(boid.O{
			T:   bkd.Lift(t),
			Tau: .9,
			F:   func(a agent.A) bool { return true },
		})
		for _, m := range mutations {
			a := m.Agent.(*config.A)
			a.SetV(v2d.Add(a.V(), v2d.Scale(frame, m.Acceleration)))
			a.SetP(v2d.Add(a.V(), v2d.Scale(frame, a.V())))

			// Model the system as a 2D toroid.
			if a.P().X() < b.Min().X(vector.AXIS_X) {
				a.SetP(*v2d.New(b.Max().X(vector.AXIS_X), a.P().Y()))
			}
			if a.P().X() > b.Max().X(vector.AXIS_X) {
				a.SetP(*v2d.New(b.Min().X(vector.AXIS_X), a.P().Y()))
			}
			if a.P().Y() < b.Min().X(vector.AXIS_Y) {
				a.SetP(*v2d.New(a.P().X(), b.Max().X(vector.AXIS_Y)))
			}
			if a.P().X() > b.Max().X(vector.AXIS_Y) {
				a.SetP(*v2d.New(a.P().X(), b.Min().X(vector.AXIS_Y)))
			}
		}

		img := image.NewPaletted(
			image.Rectangle{
				image.Point{
					int(b.Min().X(vector.AXIS_X)),
					int(b.Min().X(vector.AXIS_Y)),
				},
				image.Point{
					int(b.Max().X(vector.AXIS_X)),
					int(b.Max().X(vector.AXIS_Y)),
				},
			},
			[]color.Color{
				white,
				black,
				red,
				green,
				blue,
				gray,
			},
		)

		// Draw historical agent paths.
		for _, buf := range trailbuf {
			examplesdraw.Trail(img, margin, buf, gray)
		}

		// Draw agents.
		for _, p := range e.Points() {
			a := p.(*P).Agent()

			// Draw circle.
			examplesdraw.Circle(img, v2d.Add(margin, a.P()), int(a.R()), black)
		}

		frames = append(frames, img)

		// Update the K-D tree, as positions have changed in the
		// interim. Note that the K-D tree is mutated outside of calling
		// the Boid simulation -- this allows the tree to be used
		// elsewhere, as it's a very useful struct for other range
		// queries.
		t.Balance()
	}
	delay := make([]int, 0, *n)
	// Render with approximately 2/100 s delay, i.e. at 50Hz.
	for i := 0; i < *n; i++ {
		delay = append(delay, 2)
	}

	// Export animation as a gif.
	anim := &gif.GIF{
		Delay: delay,
		Image: frames,
	}

	w, err := os.Create(*fnOutput)
	if err != nil {
		panic(fmt.Sprintf("cannot write to file %v: %v", *fnOutput, err))
	}
	defer w.Close()

	if err := gif.EncodeAll(w, anim); err != nil {
		panic(fmt.Sprintf("cannot write to file %v: %v", *fnOutput, err))
	}
}

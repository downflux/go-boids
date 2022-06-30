package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/gif"
	"os"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/boid"
	"github.com/downflux/go-boids/demo/config"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"

	bkd "github.com/downflux/go-boids/kd"
	v2d "github.com/downflux/go-geometry/2d/vector"
	examplesdraw "github.com/downflux/go-orca/examples/draw"
)

const (
	framerate = 60.0
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
)

var (
	_ point.P = &P{}
	_ bkd.P   = &P{}
)

type P config.A

func (p *P) P() vector.V     { return vector.V(p.Agent().P()) }
func (p *P) Agent() agent.RO { return (*config.A)(p) }

type Environment struct {
	points []point.P

	height float64
	width  float64
}

func New(c config.C) *Environment {
	ps := make([]point.P, 0, len(c.Agents))
	for _, a := range c.Agents {
		p := P(*a)
		ps = append(ps, &p)
	}
	return &Environment{
		points: ps,
		height: c.Height,
		width:  c.Width,
	}
}

func (e *Environment) Points() []point.P { return e.points }

func (e *Environment) Bound() hyperrectangle.R {
	return *hyperrectangle.New(
		*vector.New(0, 0),
		*vector.New(e.width, e.height),
	)
}

func generate(fn string) *Environment {
	fp, err := os.Open(fn)
	if err != nil {
		panic(fmt.Sprintf("could not open config file: %v", err))
	}
	defer fp.Close()

	c := config.C{}
	if err := json.NewDecoder(fp).Decode(&c); err != nil {
		panic(fmt.Sprintf("could not decode config file: %v", err))
	}

	return New(c)
}

func main() {
	flag.Parse()

	e := generate(*fnInput)

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
		fmt.Fprintf(os.Stderr, "DEBUG(demo.go): frame == %v\n", i)
		// Overwrite trail buffer.
		trailbuf[i%len(trailbuf)] = nil
		for _, p := range e.Points() {
			trailbuf[i%len(trailbuf)] = append(trailbuf[i%len(trailbuf)], p.(*P).Agent().P())
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

		tau := 1.0 / float64(framerate)
		mutations := boid.Step(boid.O{
			T:   bkd.Lift(t),
			Tau: tau,
			F:   func(a agent.RO) bool { return true },
		})
		for _, m := range mutations {
			a := m.Agent.(*config.A)

			a.Locomotion(m.Steering, tau)

			// Model the system as a 2D toroid.
			x, y := a.P().X(), a.P().Y()
			if d := v2d.V(b.Min()).X() - a.P().X(); d > 0 {
				x = v2d.V(b.Max()).X() - d
			} else if d := a.P().X() - v2d.V(b.Max()).X(); d > 0 {
				x = v2d.V(b.Min()).X() + d
			}
			if d := v2d.V(b.Min()).Y() - a.P().Y(); d > 0 {
				y = v2d.V(b.Max()).Y() - d
			} else if d := a.P().Y() - v2d.V(b.Max()).Y(); d > 0 {
				y = v2d.V(b.Min()).Y() + d
			}
			a.SetP(*v2d.New(x, y))

			// Draw visualization lines.
			examplesdraw.Line(img, *segment.New(
				*line.New(a.P(), v2d.Scale(0.25, a.V())), 0, 1), black)
			examplesdraw.Line(img, *segment.New(
				*line.New(v2d.Add(a.P(), v2d.Scale(0.25, a.V())), v2d.Scale(0.25, m.Steering)), 0, 1), green)
		}

		// Draw historical agent paths.
		for _, buf := range trailbuf {
			examplesdraw.Trail(img, *v2d.New(0, 0), buf, gray)
		}

		// Draw agents.
		for _, p := range e.Points() {
			a := p.(*P).Agent()

			// Draw circle.
			examplesdraw.Circle(img, a.P(), int(a.R()), black)

			// Draw heading.
			examplesdraw.Line(img, *segment.New(
				*line.New(a.P(), polar.Cartesian(a.Heading())), 0, 2*a.R()), red,
			)
		}

		frames = append(frames, img)

		// Update the K-D tree, as positions have changed in the
		// interim. Note that the K-D tree is mutated outside of calling
		// the Boid simulation -- this allows the tree to be used
		// elsewhere, as it's a very useful struct for other range
		// queries.
		t.Balance()
	}

	// Render with approximately 2/100 s delay, i.e. at 50Hz.
	delay := make([]int, 0, *n)
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

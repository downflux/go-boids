package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/gif"
	"log"
	"os"
	"path"
	"runtime"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/agent/mock"
	"github.com/downflux/go-boids/boid"
	"github.com/downflux/go-boids/demo/config"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"

	agentdraw "github.com/downflux/go-boids/demo/draw/agent"
	bkd "github.com/downflux/go-boids/kd"
	v2d "github.com/downflux/go-geometry/2d/vector"
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
	logDir   = flag.String("log_dir", "", "")
	n        = flag.Int("frames", 1500, "")
)

var (
	_ point.P = &P{}
	_ bkd.P   = &P{}
)

type P mock.A

func (p *P) P() vector.V     { return vector.V(p.Agent().P()) }
func (p *P) Agent() agent.RO { return (*mock.A)(p) }

type Environment struct {
	points map[mock.DebugID]point.P

	height float64
	width  float64
	radius float64
}

func New(c config.C, logDir string) *Environment {
	ps := map[mock.DebugID]point.P{}
	for _, a := range c.Agents {
		fn := "/dev/null"
		if logDir != "" {
			fn = path.Join(logDir, fmt.Sprintf("%v.log", a.DebugID()))
		}
		fp, err := os.Create(fn)
		if err != nil {
			panic(fmt.Sprintf("could not create log file: %v", err))
		}
		a.Log = log.New(fp, "", log.Ldate|log.Ltime)
		p := P(*a)
		ps[a.DebugID()] = &p
	}
	return &Environment{
		points: ps,
		height: c.Height,
		width:  c.Width,
		radius: c.MaxRadius,
	}
}

func (e *Environment) Points() map[mock.DebugID]point.P { return e.points }
func (e *Environment) Data() []point.P {
	var ps []point.P
	for _, p := range e.Points() {
		ps = append(ps, p)
	}
	return ps
}

func (e *Environment) Bound() hyperrectangle.R {
	return *hyperrectangle.New(
		*vector.New(0, 0),
		*vector.New(e.width, e.height),
	)
}

func generate(fn string, logDir string) *Environment {
	fp, err := os.Open(fn)
	if err != nil {
		panic(fmt.Sprintf("could not open config file: %v", err))
	}
	defer fp.Close()

	c := config.C{}
	if err := json.NewDecoder(fp).Decode(&c); err != nil {
		panic(fmt.Sprintf("could not decode config file: %v", err))
	}

	return New(c, logDir)
}

func main() {
	flag.Parse()

	e := generate(*fnInput, *logDir)

	t, err := kd.New(e.Data())
	if err != nil {
		panic(fmt.Sprintf("could not construct tree: %v", err))
	}

	drawers := map[mock.DebugID]*agentdraw.D{}
	for _, p := range e.Data() {
		drawers[p.(*P).Agent().(*mock.A).DebugID()] = agentdraw.New(agentdraw.O{
			VelocityColor: blue,
			HeadingColor:  red,
			AgentColor:    black,
			TrailColor:    gray,
		})
	}

	b := e.Bound()

	var frames []*image.Paletted
	for i := 0; i < *n; i++ {
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

			CollisionWeight: 40,
			CollisionFilter: func(a agent.RO) bool { return true },

			ArrivalWeight: 30,

			AlignmentWeight: 20,
			AlignmentFilter: func(a agent.RO) bool { return true },

			PoolSize:  4 * runtime.GOMAXPROCS(0),
			MaxRadius: e.radius,
		})
		for _, m := range mutations {
			a := m.Agent.(*mock.A)

			agent.Step(a, m.Steering, tau)

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
			drawers[a.DebugID()].Draw(a, img)
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

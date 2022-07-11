package agent

import (
	"image"
	"image/color"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/examples/draw"
)

type A interface {
	R() float64
	P() vector.V
	V() vector.V
	Heading() polar.V
}

type O struct {
	VelocityColor color.Color
	HeadingColor  color.Color
	AgentColor    color.Color
	TrailColor    color.Color
}

func New(o O) *D {
	return &D{
		o: o,
	}
}

type D struct {
	o O

	frame       int
	trailbuffer []vector.V
}

func (d *D) Draw(a A, img *image.Paletted) {
	if d.frame < 50 {
		d.trailbuffer = append(d.trailbuffer, a.P())
	} else {
		d.trailbuffer[d.frame%len(d.trailbuffer)] = a.P()
	}

	draw.Trail(img, *vector.New(0, 0), d.trailbuffer, d.o.TrailColor)
	draw.Line(
		img,
		*segment.New(
			*line.New(
				a.P(),
				vector.Scale(0.25, a.V())),
			0,
			1,
		),
		d.o.VelocityColor,
	)
	draw.Circle(img, a.P(), int(a.R()), d.o.AgentColor)
	draw.Line(
		img,
		*segment.New(
			*line.New(
				a.P(),
				polar.Cartesian(a.Heading()),
			),
			0,
			2*a.R(),
		),
		d.o.HeadingColor,
	)

	d.frame += 1
}

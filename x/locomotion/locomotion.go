package locomotion

import (
	"fmt"
	"math"
	"os"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/accumulator"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"

	mock "github.com/downflux/go-boids/agent/mock"
)

func L(a agent.RO, steering vector.V, tau float64) agent.RO {
	v := vector.Add(a.V(), steering)

	if vector.Within(v, *vector.New(0, 0)) {
		return mock.New(mock.O{
			P:       a.P(),
			V:       *vector.New(0, 0),
			Heading: a.Heading(),
		})
	}

	if false {
		dtheta := polar.Normalize(polar.Sub(polar.Polar(v), a.Heading())).Theta()
		if dtheta >= math.Pi {
			dtheta -= 2 * math.Pi
		}

		u := *polar.New(
			math.Min(a.MaxVelocity().R(), vector.Magnitude(v)),
			a.Heading().Theta()+tau*math.Copysign(
				math.Min(
					a.MaxVelocity().Theta(),
					math.Abs(dtheta),
				),
				dtheta,
			),
		)

		return mock.New(mock.O{
			P:       vector.Add(a.P(), vector.Scale(tau, polar.Cartesian(u))),
			V:       polar.Cartesian(u),
			Heading: *polar.New(1, polar.Normalize(u).Theta()),
		})
	}

	if false {

		v, _ = accumulator.New(
			*polar.New(
				tau*a.MaxVelocity().R(),
				tau*a.MaxVelocity().Theta(),
			),
			a.Heading(),
		).Add(v)

		theta := polar.Normalize(
			polar.Sub(
				polar.Polar(v),
				a.Heading(),
			),
		).Theta()

		if theta > math.Pi {
			theta -= 2 * math.Pi
		}
		if math.Abs(theta) > math.Pi/2 {
			/*		theta = -math.Copysign(
						math.Pi - math.Abs(theta),
						theta,
					)
			*/
			fmt.Fprintf(os.Stderr, "DEBUG: theta == %v, v dot heading == %v, v == %v, heading == %v\n", theta, vector.Dot(v, polar.Cartesian(a.Heading())), v, a.Heading())
			v = vector.Scale(-1, v)
		}

	}

	u := polar.Polar(v)
	return mock.New(mock.O{
		P: vector.Add(a.P(), vector.Scale(tau, v)),
		V: v,
		Heading: polar.Normalize(
			*polar.New(
				1,
				u.Theta(),
				// a.Heading().Theta()+tau*(u.Theta()-a.Heading().Theta()),
			),
		),
	})
}

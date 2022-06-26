package polar

import (
	"math"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

type V vector.V

func New(r float64, theta float64) *V {
	v := V(*vector.New(r, theta))
	return &v
}

func (v V) R() float64 { return vector.V(v).X() }

// Theta returns the angular component of the polar coordinate. Note that theta
// may extend beyond 2π, as polar coordinates may also represent angular
// acceleration and velocity, which are not bound by a single rotation.
func (v V) Theta() float64 { return vector.V(v).Y() }

func Add(v V, u V) V               { return V(vector.Add(vector.V(v), vector.V(u))) }
func Sub(v V, u V) V               { return V(vector.Sub(vector.V(v), vector.V(u))) }
func Dot(v V, u V) float64         { return v.R() * u.R() * math.Cos(v.Theta()-u.Theta()) }
func Determinant(v V, u V) float64 { return v.R() * u.R() * math.Sin(v.Theta()-u.Theta()) }

func Polar(v vector.V) V {
	return V(*vector.New(
		math.Sqrt(v.X()*v.X()+v.Y()*v.Y()),
		math.Atan2(v.Y(), v.X()),
	))
}

// Normalize returns a vector whose anglular component is bound between 0 and
// 2π.
func Normalize(v V) V {
	theta := math.Mod(v.Theta(), 2*math.Pi)
	// theta may be negative in the case the original polar coordinate is
	// negative. Since we want to ensure the angle is positive, we have to
	// take this into consideration.
	if theta < 0 {
		theta += 2 * math.Pi
	}
	return *New(v.R(), theta)
}

func Cartesian(v V) vector.V {
	return *vector.New(
		v.R()*math.Cos(v.Theta()),
		v.R()*math.Sin(v.Theta()),
	)
}

func WithinEpsilon(v V, u V, e epsilon.E) bool {
	return (e.Within(v.R(), 0) && e.Within(u.R(), 0)) || vector.WithinEpsilon(vector.V(v), vector.V(u), e)
}
func Within(v, u V) bool { return WithinEpsilon(v, u, epsilon.DefaultE) }

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

func (v V) R() float64     { return vector.V(v).X() }
func (v V) Theta() float64 { return vector.V(v).Y() }

func Add(v V, u V) V { return V(vector.Add(vector.V(v), vector.V(u))) }
func Sub(v V, u V) V { return V(vector.Sub(vector.V(v), vector.V(u))) }

func Polar(v vector.V) V {
	return V(*vector.New(
		math.Sqrt(v.X()*v.X()+v.Y()*v.Y()),
		math.Atan2(v.Y(), v.X()),
	))
}

func WithinEpsilon(v V, u V, e epsilon.E) bool {
	if !e.Within(v.R(), u.R()) {
		return false
	}
	angle := v.Theta() - u.Theta()
	if !e.Within(
		angle/(2*math.Pi),
		math.Round(angle/(2*math.Pi)),
	) && !e.Within(v.R(), 0) && !e.Within(u.R(), 0) {
		return false
	}
	return true
}

func Within(v, u V) bool { return WithinEpsilon(v, u, epsilon.DefaultE) }

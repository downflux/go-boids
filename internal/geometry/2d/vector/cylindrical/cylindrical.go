package cylindrical

import (
	"github.com/downflux/go-geometry/2d/vector"
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

import numpy as np

from underactuated import PyPlotVisualizer
from pydrake.systems.framework import BasicVector_, LeafSystem_, PortDataType
from pydrake.systems.scalar_conversion import TemplateSystem


# Note: In order to use the Python system with drake's autodiff features, we
# have to add a little "TemplateSystem" boilerplate (for now).  For details,
# see https://drake.mit.edu/pydrake/pydrake.systems.scalar_conversion.html

# TODO(russt): Clean this up pending any resolutions on
#  https://github.com/RobotLocomotion/drake/issues/10745
@TemplateSystem.define("Quadrotor2D_")
def Quadrotor2D_(T):

    class Impl(LeafSystem_[T]):
        def _construct(self, converter=None):
            LeafSystem_[T].__init__(self, converter)
            # two inputs (thrust)
            self.DeclareVectorInputPort("u", BasicVector_[T](2))
            # six outputs (full state)
            self.DeclareVectorOutputPort("x", BasicVector_[T](4),
                                         self.CopyStateOut)
            # three positions, three velocities
            self.DeclareContinuousState(2, 2, 0)

            # parameters based on [Bouadi, Bouchoucha, Tadjine, 2007]
            #self.length = 0.25      # length of rotor arm
            self.mass = 0.42       # mass of quadrotor
            #self.inertia = 0.00383  # moment of inertia
            self.gravity = 9.81     # gravity

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, converter=converter)

        def CopyStateOut(self, context, output):
            x = context.get_continuous_state_vector().CopyToVector()
            y = output.SetFromVector(x)

        def DoCalcTimeDerivatives(self, context, derivatives):
            x = context.get_continuous_state_vector().CopyToVector()
            u = self.EvalVectorInput(context, 0).CopyToVector()
            q = x[:2]
            qdot = x[2:]
            qddot = np.array([self.mass*self.gravity*np.cos(0)*np.tan(u[0])-self.mass*self.gravity*sin(0)*tan(u[1]),
                              self.mass*self.gravity*sin(0)*tan(u[1])+self.mass*self.gravity*cos(0)*tan(u[1])])
            derivatives.get_mutable_vector().SetFromVector(
                np.concatenate((qdot, qddot)))

    return Impl


bebop2d = Quadrotor2D_[None]  # Default instantiation
